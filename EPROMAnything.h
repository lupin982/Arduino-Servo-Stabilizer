/*
 * EPROMAnything.h
 *
 * Created: 12/01/2015 23:12:40
 *  Author: 
 */ 


#ifndef EPROMANYTHING_H_
#define EPROMANYTHING_H_



template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
	const byte* p = (const byte*)(const void*)&value;
	unsigned int i;
	for (i = 0; i < sizeof(value); i++)
	EEPROM.write(ee++, *p++);
	return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
	byte* p = (byte*)(void*)&value;
	unsigned int i;
	for (i = 0; i < sizeof(value); i++)
	*p++ = EEPROM.read(ee++);
	return i;
}


#endif /* EPROMANYTHING_H_ */