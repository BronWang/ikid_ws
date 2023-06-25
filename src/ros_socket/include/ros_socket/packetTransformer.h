#ifndef PACKETTRANSFORMER_H
#define PACKETTRANSFORMER_H
#include "ros_socket/ByteArray.h"
#include "ros_socket/def.h"
#include "ros_socket/serialPacket.h"
#include <cstring>

#define MAX_BUFFER_LENGTH	1024

class PacketTransformer
{
public:
    DSP_INST_PACKET *dspInst;
public:
    PacketTransformer();
    PacketTransformer(PBYTE buf,int len);
    void SetRawPacket(PBYTE buf,int len);
    void SetRawPacket(ByteArray &recv);
    void Reset();
    //void AppandArray(ByteArray &recv);

    bool ConstructPacket();
    bool DestructPacket();
    bool TryDestructOnePacket();
    ByteArray& GetByteArray();
    BYTE CheckSum();
    int GetLength()
    {
        return length;
    };

protected:
    ByteArray result;
    int length;
    BYTE buffer[MAX_BUFFER_LENGTH];
};

#endif // PACKETTRANSFORMER_H
