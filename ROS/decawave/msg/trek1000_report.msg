Header header
#message ID, r for raw tag to anchor, c for range bias corrected tag to anchor, a for range bias corrected anchor to anchor ranges
char mID
#uint8 mID
#valid ranges in binary e.g. 1111 = 0xF is all ranges valid range0 is LSB
uint8 mask
#ranges are reported in units of millimeters
int32[4] range
#this is a number of ranges completed by reporting unit raw range
uint16 nRanges
#range sequence number
uint8 seq
#if mID = r/c: time of last range reported, if mID = a: TX/RX antenna delays as two 16-bit numbers
uint32 debug
#report source: a for anchor or t for tag (source ID stays constant for all reports while the communictor ID varies)
uint8 c
#anchor source/communicator ID
uint8 aID
#tag source/communicator ID (mID = a: always returns 0 for tID)
uint8 tID
