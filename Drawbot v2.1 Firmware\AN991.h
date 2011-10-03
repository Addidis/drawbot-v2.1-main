//Global variables 

extern unsigned int PageSize;

//Function Prototypes

unsigned char putstringI2C( unsigned char *);
void InitPIC(void);         // Initializes the PIC to POR defaults

unsigned char LDByteWriteI2C( unsigned char, unsigned char, unsigned char );
unsigned char LDByteReadI2C(unsigned char, unsigned char, unsigned char*, unsigned char);
unsigned char LDPageWriteI2C(unsigned char, unsigned char, unsigned char *);
unsigned char LDSequentialReadI2C(unsigned char, unsigned char, unsigned char *, unsigned char);

unsigned char HDByteWriteI2C(unsigned char, unsigned char, unsigned char, unsigned char);
unsigned char HDByteReadI2C(unsigned char, unsigned char, unsigned char, unsigned char*, unsigned char);
unsigned char HDPageWriteI2C( unsigned char, unsigned char, unsigned char, unsigned char *);
unsigned char HDSequentialReadI2C(unsigned char, unsigned char, unsigned char, unsigned char *, unsigned char );

//********************************************************************
//Constant Definitions
//********************************************************************


