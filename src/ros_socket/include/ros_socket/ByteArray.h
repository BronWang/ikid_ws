// created by Anod

#ifndef __BYTEARRAY_H_
#define __BYTEARRAY_H_

#include <cstdint>
#include <cstddef>
#include <cstdio>

#include <vector>
#include <stdexcept>
#include <functional>

#ifndef BYTEARRAY_RESERVE_SIZE
#define BYTEARRAY_RESERVE_SIZE 128		//每个ByteArray在构造的时候预先分配多少内存，单位(字节)
#endif

#undef LITTLE_ENDIAN
#undef BIG_ENDIAN

enum Endian{
	LITTLE_ENDIAN,
	BIG_ENDIAN
};

class ByteArray
{
public:
	static Endian getNativeEndian();

public:
	/**
	 * reserve_size: 预先分配内存大小，单位（字节）
	 * endian: 字节序，LITTLE_ENDIAN/BIG_ENDIAN以外的值表示使用本地机器字节序
	 */
	ByteArray( size_t reserve_size=BYTEARRAY_RESERVE_SIZE, int endian=-1 );

	virtual ~ByteArray();

public: // 类成员 setter and getter
	inline size_t		getLength() const;
	void			setLength( size_t );
	
	inline Endian		getEndian() const;
	inline void		setEndian( Endian e );

	inline int		getPosition() const;
	inline void		setPosition( size_t p );
	
	inline uint8_t*		getBuffer();				//拿缓冲区指针
	inline const uint8_t*	getConstBuffer() const;			//拿缓冲区指针，但是是const指针

public: // 缓冲区数据访问
	inline uint8_t		at( size_t position ) const;		//越界异常的访问方式
	inline int		operator[]( size_t position ) const;	//非越界异常的访问方式

	/**
	 * 直接写入缓冲区，和as3不同，不提供设置下标的能力，而是
	 * 替换为一个函数来处理
	 */
	inline void		assign( size_t position, uint8_t value );

	inline void		clear();				//缓冲区清空 
	void			resize( size_t length );	//重新分配数组大小
	void			cut( size_t position, size_t length );	//把position,position+length的数据留下，其他删除

public: // IDataInput的实现
	inline size_t		getBytesAvailable() const;
	
	bool 			readBoolean();
	int8_t 			readByte();
	double 			readDouble();
	float 			readFloat();
	int32_t 		readInt();
	int16_t 		readShort();
	uint8_t 		readUnsignedByte();
	uint32_t 		readUnsignedInt();
	uint16_t 		readUnsignedShort();
	void 			readBytes( void *bytes, uint32_t offset, size_t length );
	int64_t 		readLongLong();
	uint64_t 		readUnsignedLongLong();

public: // 扩展IDataInput
	inline void		readBytes( ByteArray *bytes, uint32_t offset=0, size_t length=0 );

	/**
	 * 作为缓冲区使用，把从position下标开始，长度为length的数据传入func
	 */
	inline void		readBytesToCall( std::function<void (const void*, size_t)> func, size_t length );

public: // IDataOutput实现
	void 			writeBoolean( bool value );
	void 			writeByte( int8_t value );
	void 			writeDouble( double value );
	void 			writeFloat( float value );
	void 			writeShort( int16_t value );
	void 			writeInt( int32_t value );
	void 			writeUnsignedByte( uint8_t value );
	void 			writeUnsignedShort( uint16_t value );
	void 			writeUnsignedInt( uint32_t value );
	void 			writeBytes( const void *bytes, uint32_t offset, size_t length );
	void 			writeLongLong( int64_t value );
	void 			writeUnsignedLongLong( uint64_t value );

public: // 扩展IDataOutput
	inline void 		writeBytes( const ByteArray *bytes, uint32_t offset=0, size_t length=0 );

	/**
	 * 作为缓冲区使用，从func中拉取长度为length数据的数据，写入到position开始的位置
	 */
	inline int  		writeBytesFromCall( std::function<int (void*, size_t)> func, size_t length );

protected:
	std::vector<uint8_t> _buffer;
	Endian _endian;
	uint32_t _position = 0;
};

//////////////////////////////////////////
//        code for inline method
//////////////////////////////////////////

inline size_t ByteArray::getLength() const
{
	return _buffer.size();
}

inline Endian ByteArray::getEndian() const
{
	return _endian;
}

inline void	ByteArray::setEndian( Endian e )
{
	this->_endian = e;
}

inline int ByteArray::getPosition() const
{
	return _position;
}

inline void	ByteArray::setPosition( size_t p )
{
	if( p < this->getLength() ) this->_position = p;
	else this->_position = this->getLength();
}

inline uint8_t ByteArray::at( size_t position ) const
{
	return this->_buffer.at( position );
}

inline int ByteArray::operator[]( size_t position ) const
{
	if( position >= getLength() ) return EOF;
	else return this->_buffer[position];
}

inline void ByteArray::assign( size_t position, uint8_t value )
{
	if( position >= getLength() ) //缓冲区扩张
		this->_buffer.resize( position+1, 0 );
	this->_buffer[position] = value;
}

inline void ByteArray::clear()
{
	this->_buffer.clear();
	this->_position = 0;
}

inline uint8_t* ByteArray::getBuffer()
{
	return this->_buffer.data();
}

inline const uint8_t* ByteArray::getConstBuffer() const
{
	return this->_buffer.data();
}

inline size_t ByteArray::getBytesAvailable() const
{
	return this->getLength() - this->getPosition();
}

inline void ByteArray::readBytes( ByteArray *bytes, uint32_t offset, size_t length )
{
	if( length == 0 ) length = getBytesAvailable();
	if( bytes->getLength() < offset + length ) bytes->setLength( offset + length );
	this->readBytes( bytes->_buffer.data(), offset, length );
}

inline void	ByteArray::readBytesToCall( std::function<void (const void*, size_t)> func, size_t length )
{
	size_t available_bytes = this->getBytesAvailable();
	if( available_bytes < length )
		length = available_bytes;

	func( this->_buffer.data() + this->_position, length );
	this->setPosition( this->getPosition() + length );
}

inline void ByteArray::writeBytes( const ByteArray *bytes, uint32_t offset, size_t length )
{
	if( length == 0 ) length = bytes->getLength() - offset;
	if( bytes->getLength() < offset + length ) length = bytes->getLength() - offset;
	this->writeBytes( bytes->_buffer.data(), offset, length );
}

inline int  ByteArray::writeBytesFromCall( std::function<int (void*, size_t)> func, size_t length )
{
	uint32_t size_before = this->_buffer.size();

	if( this->getBytesAvailable() < length )		//保证能放下length的数据
		this->_buffer.resize(this->_position+length, 0);

	int retval = func( this->_buffer.data() + this->_position, length );
	this->setPosition( this->getPosition() + retval );
	this->_buffer.resize( std::max(this->_position, size_before) );
	return retval;
}

#endif//_BYTEARRAY_H_