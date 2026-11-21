#include "ring_link.h"

#ifndef RING_HOST_TEST
#include "zf_common_headfile.h"
#pragma location = RING_COMMAND_ADDRESS
__root __no_init int32_t ring_command_mailbox[RING_LINK_WORDS];
#pragma location = RING_MOTION_ADDRESS
__root __no_init int32_t ring_motion_mailbox[RING_LINK_WORDS];
#pragma location = RING_DIAGNOSTIC_ADDRESS
__root __no_init int32_t ring_diagnostic_mailbox[RING_LINK_WORDS];
#pragma location = RING_ENTRY_DIAGNOSTIC_ADDRESS
__root __no_init int32_t ring_entry_diagnostic_mailbox[RING_LINK_WORDS];
#pragma location = RING_GEOMETRY_DIAGNOSTIC_ADDRESS
__root __no_init int32_t ring_geometry_diagnostic_mailbox[RING_LINK_WORDS];

#pragma location = RING_TOPOLOGY_DIAGNOSTIC_ADDRESS
__root __no_init int32_t ring_topology_diagnostic_mailbox[RING_LINK_WORDS];
#pragma location = RING_MOUTH_DIAGNOSTIC_ADDRESS
__root __no_init int32_t ring_mouth_diagnostic_mailbox[RING_LINK_WORDS];
#pragma location = RING_EXIT_DIAGNOSTIC_ADDRESS
__root __no_init int32_t ring_exit_diagnostic_mailbox[RING_LINK_WORDS];
#pragma location = RING_IMAGE_ADDRESS
__root __no_init ring_image_packet_t ring_image_mailbox;
#pragma location = RING_IMAGE_ACK_ADDRESS
__root __no_init int32_t ring_image_ack_mailbox[RING_LINK_WORDS];

static void image_clean(void)
{ SCB_CleanDCache_by_Addr((uint32_t *)&ring_image_mailbox, 128); __DMB(); }
static void image_invalidate(void)
{ SCB_InvalidateDCache_by_Addr((uint32_t *)&ring_image_mailbox, 128); __DMB(); }
static void barrier(void) { __DMB(); }
static void clean(int32_t *p)
{
    SCB_CleanDCache_by_Addr((uint32_t *)p, 32);
    __DMB();
}
static void invalidate(int32_t *p)
{
    SCB_InvalidateDCache_by_Addr((uint32_t *)p, 32);
    __DMB();
}
#else
int32_t ring_command_mailbox[RING_LINK_WORDS];
int32_t ring_motion_mailbox[RING_LINK_WORDS];
int32_t ring_diagnostic_mailbox[RING_LINK_WORDS];
int32_t ring_entry_diagnostic_mailbox[RING_LINK_WORDS];
int32_t ring_geometry_diagnostic_mailbox[RING_LINK_WORDS];
int32_t ring_topology_diagnostic_mailbox[RING_LINK_WORDS];
ring_image_packet_t ring_image_mailbox;
int32_t ring_mouth_diagnostic_mailbox[RING_LINK_WORDS];
int32_t ring_exit_diagnostic_mailbox[RING_LINK_WORDS];
int32_t ring_image_ack_mailbox[RING_LINK_WORDS];
static void image_clean(void) { }
static void image_invalidate(void) { }
static void barrier(void) { }
static void clean(int32_t *p) { (void)p; }
static void invalidate(int32_t *p) { (void)p; }
#endif

void ring_link_write(int32_t *destination, const ring_packet_t *packet,
                     uint32_t *sequence)
{
    volatile int32_t *out = destination;
    uint32_t next = *sequence + 2u;
    unsigned i;
    if (!next) next = 2u;
    out[RL_SEQUENCE] = (int32_t)(next - 1u);
    barrier();
    clean(destination);
    for (i = 0; i < RING_LINK_WORDS; ++i)
        if (i != RL_SEQUENCE) out[i] = packet->word[i];
    barrier();
    clean(destination);
    out[RL_SEQUENCE] = (int32_t)next;
    clean(destination);
    *sequence = next;
}

static int read_expected(int32_t *source, ring_packet_t *packet, int32_t magic)
{
    ring_packet_t copy;
    volatile const int32_t *in = source;
    uint32_t before, after;
    unsigned i;
    invalidate(source); /* A reader never writes back another core's line. */
    before = (uint32_t)in[RL_SEQUENCE];
    if (!before || (before & 1u)) return 0;
    for (i = 0; i < RING_LINK_WORDS; ++i) copy.word[i] = in[i];
    barrier();
    invalidate(source);
    after = (uint32_t)in[RL_SEQUENCE];
    if (before != after || copy.word[RL_MAGIC] != magic) return 0;
    *packet = copy;
    return 1;
}

int ring_link_read(int32_t *source, ring_packet_t *packet)
{ return read_expected(source, packet, RING_LINK_MAGIC); }
int ring_link_read_diagnostic(ring_packet_t *packet)
{ return read_expected(ring_diagnostic_mailbox, packet, RING_DIAGNOSTIC_MAGIC); }
int ring_link_read_entry_diagnostic(ring_packet_t *packet)
{ return read_expected(ring_entry_diagnostic_mailbox, packet, RING_ENTRY_DIAGNOSTIC_MAGIC); }
int ring_link_read_geometry_diagnostic(ring_packet_t *packet)
{ return read_expected(ring_geometry_diagnostic_mailbox, packet, RING_GEOMETRY_DIAGNOSTIC_MAGIC); }

int ring_link_read_topology_diagnostic(ring_packet_t *packet)
{ return read_expected(ring_topology_diagnostic_mailbox, packet, RING_TOPOLOGY_DIAGNOSTIC_MAGIC); }

void ring_link_write_image(const ring_image_packet_t *packet, uint32_t *sequence)
{
    volatile ring_image_packet_t *out = &ring_image_mailbox;
    uint32_t next=*sequence+2u;
    unsigned i;
    if (!next) next=2u;
    out->word[RL_SEQUENCE]=(int32_t)(next-1u);
    barrier(); image_clean();
    for(i=0;i<RING_IMAGE_HEADER_WORDS;++i)
        if(i!=RL_SEQUENCE) out->word[i]=packet->word[i];
    for(i=0;i<RING_IMAGE_CHUNK_BYTES;++i) out->data[i]=packet->data[i];
    barrier(); image_clean();
    out->word[RL_SEQUENCE]=(int32_t)next;
    image_clean(); *sequence=next;
}
int ring_link_read_image(ring_image_packet_t *packet)
{
    volatile const ring_image_packet_t *in=&ring_image_mailbox;
    ring_image_packet_t copy;
    uint32_t before,after;
    unsigned i;
    image_invalidate(); before=(uint32_t)in->word[RL_SEQUENCE];
    if(!before || (before&1u)) return 0;
    for(i=0;i<RING_IMAGE_HEADER_WORDS;++i) copy.word[i]=in->word[i];
    for(i=0;i<RING_IMAGE_CHUNK_BYTES;++i) copy.data[i]=in->data[i];
    barrier(); image_invalidate(); after=(uint32_t)in->word[RL_SEQUENCE];
    if(before!=after || copy.word[RL_MAGIC]!=RING_IMAGE_MAGIC) return 0;
    *packet=copy; return 1;
}
void ring_link_ack_image(const ring_image_packet_t *packet,uint32_t *sequence)
{
    ring_packet_t ack={{0}};
    ack.word[RL_MAGIC]=RING_IMAGE_ACK_MAGIC;
    ack.word[RL_EPOCH]=packet->word[RL_EPOCH];
    ack.word[RA_IMAGE_SEQUENCE]=packet->word[RL_SEQUENCE];
    ack.word[RA_ID]=packet->word[RI_ID];
    ack.word[RA_OFFSET_LENGTH]=packet->word[RI_OFFSET_LENGTH];
    ring_link_write(ring_image_ack_mailbox,&ack,sequence);
}
int ring_link_read_image_ack(ring_packet_t *packet)
{ return read_expected(ring_image_ack_mailbox,packet,RING_IMAGE_ACK_MAGIC); }
uint32_t ring_image_crc32(const uint8_t *data,unsigned length)
{
    uint32_t crc=0xffffffffu;
    unsigned i,bit;
    for(i=0;i<length;++i) {
        crc^=data[i];
        for(bit=0;bit<8u;++bit) crc=(crc>>1)^((crc&1u)?0xedb88320u:0u);
    }
    return ~crc;
}

int ring_link_read_mouth_diagnostic(ring_packet_t *packet)
{ return read_expected(ring_mouth_diagnostic_mailbox,packet,RING_MOUTH_DIAGNOSTIC_MAGIC); }
int ring_link_read_exit_diagnostic(ring_packet_t *packet)
{ return read_expected(ring_exit_diagnostic_mailbox,packet,RING_EXIT_DIAGNOSTIC_MAGIC); }
