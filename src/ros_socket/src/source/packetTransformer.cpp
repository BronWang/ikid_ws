#include "ros_socket/packetTransformer.h"
#include "ros_socket/serialPacket.h"

PacketTransformer::PacketTransformer()
{
    dspInst=(DSP_INST_PACKET *)buffer;
    length=0;
}

PacketTransformer::PacketTransformer(PBYTE buf,int len)
{
    dspInst=(DSP_INST_PACKET *)buffer;
    memcpy(PBYTE(&buffer),buf,len);
    length=len;
}

void PacketTransformer::SetRawPacket(PBYTE buf,int len)
{
    result.clear();
    memcpy(PBYTE(&buffer),buf,len);
    for(int i=0;i<len;i++)
        result.assign(i,buffer[i]);
    length=len;
    
}

void PacketTransformer::SetRawPacket(ByteArray &recv)
{
    result.clear();
    result.writeBytes(&recv, recv.getLength());
}

ByteArray& PacketTransformer::GetByteArray()
{
    return result;
}

bool PacketTransformer::ConstructPacket()
{
    int i=0;

    result.clear();
    result.resize(dspInst->length+4);
    result.assign(0, 0xff);
    result.assign(1, 0xff);
    for(i=0;i<dspInst->length+1;i++)
        result.assign(i+2,buffer[i]);
    result.assign(i+2, CheckSum());
    return true;

}

bool PacketTransformer::DestructPacket()
{
    PBYTE pbuf=NULL;
    if(result.getLength()<6){

        return false;
    }
    //result.readBytes(pbuf,0,result.getLength());
    pbuf=reinterpret_cast<PBYTE>(result.getBuffer());
    if(
        0xff!=pbuf[0]||
        0xff!=pbuf[1]
        ){
        return false;
    }
    memcpy(buffer,pbuf+2,result.getLength()-3);
    if(
        dspInst->id!=ID_DSP||
        dspInst->length!=result.getLength()-4||
        pbuf[result.getLength()-1]!=CheckSum()
        )
        return false;
    return true;
}

bool PacketTransformer::TryDestructOnePacket()
{
    PBYTE pbuf=NULL;
    DSP_INST_PACKET *ppacket=NULL;

    while(result.getLength()>=6)
    {
        //result.readBytes(pbuf,0,result.getLength());
        pbuf=reinterpret_cast<PBYTE>(result.getBuffer());
        if(pbuf[0]!=0xff)
        {
            result.cut(1,result.getLength()-1);
            continue;
        }
        if(pbuf[1]!=0xff)
        {
            result.cut(2,result.getLength()-2);
            continue;
        }
        ppacket=(DSP_INST_PACKET *)(pbuf+2);
        if(ppacket->id!=ID_DSP)
        {
            result.cut(3,result.getLength()-3);
            continue;
        }
        if(result.getLength()<ppacket->length+4)
            return false;
        memcpy(buffer,ppacket,ppacket->length+2);
        result.cut(dspInst->length+4,result.getLength()-(dspInst->length+4));
        if(buffer[ppacket->length+1]!=CheckSum())
            continue;
        return true;
    }
    return false;
}

BYTE PacketTransformer::CheckSum()
{
    BYTE checkSum=0;
    for(int i=0;i<dspInst->length+1;i++)
        checkSum+=buffer[i];
    checkSum=~checkSum;
    return checkSum;
}

void PacketTransformer::Reset()
{
    result.clear();
    result.resize(0);
}

// void PacketTransformer::AppandArray(ByteArray &recv)
// {
//     result.append(recv);
// }
