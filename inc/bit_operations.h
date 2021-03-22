#pragma once

template<class T> inline T set_bit(T num, unsigned char position)
{
	return num | (1 << position);
}

template<class T> inline bool get_bit(T num, unsigned char position)
{
	return (num & (1 << position)) >> position;
}

template<class T> inline T clear_bit(T num, unsigned char position)
{
	return num & ~(1 << position);
}