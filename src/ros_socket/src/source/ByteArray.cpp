#include "ros_socket/ByteArray.h"
#include <algorithm>
#include <cstring>

static Endian nativeEndian;

/**
 * 得到本机的字节序，算法参考：http://blog.csdn.net/lincyang/article/details/17266027
 */
Endian ByteArray::getNativeEndian()
{
	static bool done = false;
	if( !done )
	{
		int x = 1;
		if( *(char*)&x == 1 ) 
			nativeEndian = LITTLE_ENDIAN;
		else nativeEndian = BIG_ENDIAN;
		done = true;
	}
	return nativeEndian;
}

inline void adjustEndian( uint8_t *buffer, size_t size, Endian endian )
{
	if( endian != nativeEndian && size > 1 )
	{
		std::reverse( buffer, buffer+size );
    }
}

static bool nativeEndianDone = false;
ByteArray::ByteArray( size_t size, int endian )
	:_buffer(), _position(0)
{
	Endian native = getNativeEndian();
	if( endian == BIG_ENDIAN ) _endian = BIG_ENDIAN;
	else if( endian == LITTLE_ENDIAN ) _endian = LITTLE_ENDIAN;
	else _endian = native;

	if( size > 0 ) 
	{
		_buffer.reserve( size );
	}
}

ByteArray::~ByteArray()
{
	_buffer.clear();
}

void ByteArray::resize(size_t length ){
    _buffer.resize(length);
}

void ByteArray::cut( size_t position, size_t length )
{
	size_t size = this->getLength();
	if( position + length > size )
		length = size - position;

	if( position < size && length > 0 ) 
	{
		if( position > 0 )
		{
			auto pSrc = _buffer.begin() + position;
			std::move(pSrc, pSrc+length, _buffer.begin());
			_buffer.resize( length );
		}
		else if( length < size )
			_buffer.resize( length );
		
		if( length < this->_position ) this->_position = length;
	}
	else this->clear();
}

/**
 * 在as3中，如果length比当前值要大，则往缓冲区中填充0
 * 如果length和当前值一致，则什么都不做
 * 如果length比当前值小，则截断数据到length
 */
void ByteArray::setLength( size_t length )
{
	size_t _length = this->getLength();

	if( length == _length ) return;
	else _buffer.resize( length, 0 );

	if( length < _position ) _position = length;
}

#define RANGE_CHECK( nBytes ) if( this->getBytesAvailable() < (nBytes) ) throw std::out_of_range("end of buffer");

bool ByteArray::readBoolean()
{
	RANGE_CHECK( sizeof(bool) );

	return at(this->_position++) != 0;
}

uint8_t ByteArray::readUnsignedByte()
{
	RANGE_CHECK( sizeof(uint8_t) );

	return at( this->_position++ );
}

/**
 * 写入缓冲区bytes，起始地址&bytes[offset]，从buffer的position开始写，长度是length
 */
void ByteArray::readBytes( void *bytes, uint32_t offset, size_t length )
{
	RANGE_CHECK( length );

	uint8_t *p_data_src = static_cast<uint8_t*>(bytes);
	p_data_src += offset;

	memcpy( p_data_src, _buffer.data() + this->_position, length );
	this->_position += length;
}

//其实是不是可以用模板呢？相信肯定是可以的，这个模板请尽量inline。可惜作者水平不精，不懂写，见笑了
#define READ_BUILDIN_TEMPLATE( type ) \
	size_t size = sizeof(type); \
	RANGE_CHECK( size ); \
	\
	type retval; \
	retval = * ( reinterpret_cast<type*>(_buffer.data()+this->_position) ); \
	adjustEndian( reinterpret_cast<uint8_t*>( &retval ), size, this->_endian ); \
	\
	_position += size; \
	\
	return retval; 

double ByteArray::readDouble()
{
	READ_BUILDIN_TEMPLATE( double );
}

float ByteArray::readFloat()
{
	READ_BUILDIN_TEMPLATE( float );
}

int32_t ByteArray::readInt()
{
	READ_BUILDIN_TEMPLATE( int32_t );
}

int64_t ByteArray::readLongLong()
{
	READ_BUILDIN_TEMPLATE( int64_t );
}

int16_t ByteArray::readShort()
{
	READ_BUILDIN_TEMPLATE( int16_t );
}

int8_t ByteArray::readByte()
{
	READ_BUILDIN_TEMPLATE( int8_t );
}

uint16_t ByteArray::readUnsignedShort()
{
	READ_BUILDIN_TEMPLATE( uint16_t );
}

uint32_t ByteArray::readUnsignedInt()
{
	READ_BUILDIN_TEMPLATE( uint32_t );
}

uint64_t ByteArray::readUnsignedLongLong()
{
	READ_BUILDIN_TEMPLATE( uint64_t );
}

#define RANGE_RESERVE( nBytes ) if( this->getBytesAvailable() < nBytes ) this->_buffer.resize( this->_position+nBytes, 0 ); 

void ByteArray::writeBoolean( bool value )
{
	_buffer.push_back( static_cast<uint8_t>(value) );
	++this->_position;
}

void ByteArray::writeByte( int8_t value )
{
	_buffer.push_back( *( reinterpret_cast<uint8_t*>(&value) ) );
	++this->_position;
}

void ByteArray::writeUnsignedByte( uint8_t value )
{
	_buffer.push_back( value );
	++this->_position;
}

/**
 * 写入缓冲区buffer，起始地址是&buffer[position]，写入长度length，来源地址&bytes[offset]
 */
void ByteArray::writeBytes( const void *bytes, uint32_t offset, size_t length )
{
	RANGE_RESERVE( length );

	const uint8_t *src = static_cast<const uint8_t*>(bytes) + offset;
	memcpy( _buffer.data()+this->_position, src, length );
	this->_position += length;
}

#define WRITE_BUILDIN_TEMPLATE( type, value ) \
	size_t size = sizeof( type ); \
	RANGE_RESERVE( size ); \
	\
	uint8_t *dest = _buffer.data() + this->_position; \
	*( reinterpret_cast<type*>(dest) ) = value; \
	adjustEndian( dest, size, this->_endian ); \
	\
	this->_position += size;

void ByteArray::writeDouble( double value )
{
	WRITE_BUILDIN_TEMPLATE( double, value );
}

void ByteArray::writeFloat( float value )
{
	WRITE_BUILDIN_TEMPLATE( float, value );
}

void ByteArray::writeInt( int32_t value )
{
	WRITE_BUILDIN_TEMPLATE( int32_t, value );
}

void ByteArray::writeLongLong( int64_t value )
{
	WRITE_BUILDIN_TEMPLATE( int64_t, value );
}

void ByteArray::writeShort( int16_t value )
{
	WRITE_BUILDIN_TEMPLATE( int16_t, value );
}

void ByteArray::writeUnsignedShort( uint16_t value )
{
	WRITE_BUILDIN_TEMPLATE( uint16_t, value );
}

void ByteArray::writeUnsignedInt( uint32_t value )
{
	WRITE_BUILDIN_TEMPLATE( int32_t, value );
}

void ByteArray::writeUnsignedLongLong( uint64_t value )
{
	WRITE_BUILDIN_TEMPLATE( uint64_t, value );
}