#define FLAG 0x7E
//Protocol Identifier. 0xF0 means : No Layer 3 protocol implemented
#define PID 0xF0
//Control Field Type for Unnumbered Information Frames : 0x03
#define CONTROL 0x03
#define MAX_LENGTH 280
#define MAX_LENGTH_FINAL 450
//CRC-CCITT
#define CRC_POLYGEN     0x1021

char SrcCallsign[7] = "KD2BHC";
char DestCallsign[7] = "CQ    ";

byte ssid_source = 0x61;
byte ssid_destination = 0x60;

byte bitSequence[280*8];
byte finalSequence[450];

int Index = 0;

// CRC-CCITT
unsigned int FCS = 0;

void AddHeader(byte *Buffer);
void BitProcessing(byte *Buffer,uint8_t bytelength);

//CRC_CCITT
unsigned int CRC_CCITT (byte *Buffer,uint8_t bytelength);
boolean logicXOR(boolean a, boolean b);
unsigned int MSB_LSB_swap_16bit(unsigned int v);
byte MSB_LSB_swap_8bit(byte v);

int ax25encode(char* message, int message_len)
{  
  //Array Initialization
  Index = 0;
  for (int i=0; i< MAX_LENGTH * 8 ;i++) bitSequence[i] = 0;
  for (int i=0; i< MAX_LENGTH_FINAL ;i++) finalSequence[i] = 0;
  
  //----------------------- Start Ax25 Packet format ----------------------//
  
  //Add Header
  AddHeader(bitSequence);
      
  //Add Message
  for (int i=0; i < message_len ; i++) bitSequence[Index++] = message[i];
     
  //Convert bit sequence from MSB to LSB
  for (int i=0; i < Index ; i++) bitSequence[i] = MSB_LSB_swap_8bit(bitSequence[i]);
    
  //Compute Frame check sequence : CRC
  FCS = CRC_CCITT(bitSequence, Index);
  
  //Add FCS in MSB form
  //Add MS byte
  bitSequence[Index++] = (FCS >> 8) & 0xff;
  //Add LS byte
  bitSequence[Index++] = FCS & 0xff;
    
  //radio.printBuffer("Init Message:", bitSequence, Index);
    
  //Bit Processing...Bit stuff, add FLAG and do NRZI enconding...
  BitProcessing(bitSequence,Index);

  return Index;
}

void AddHeader(byte *Buffer)
{
    //Shift bits 1 place to the left in order to allow for HDLC extension bit
    for (int i=0; i < strlen(DestCallsign) ; i++) Buffer[Index++] = DestCallsign[i]<<1;

    // Append SSID Destination
    Buffer[Index++] = ssid_destination;  

    //Append Source Callsign
    for (int i=0; i < strlen(SrcCallsign) ; i++) Buffer[Index++] = SrcCallsign[i]<<1;
    
    //Append SSID Source
    Buffer[Index++] = ssid_source;
   
    //Append Control bits
    Buffer[Index++] = CONTROL;
    
    //Append Protocol Identifier
    Buffer[Index++] = PID;
}

void BitProcessing(byte *Buffer, uint8_t bytelength)
{ 
    byte BitSequence[bytelength*8+1];
    byte BitSequenceStuffed[bytelength*8+bytelength*8/5+1];
    int k = 0; //general counter
    int _size = 0;
    int s = 0; //stuffed sequence counter
    uint8_t cnt = 0 ;//Bit stuff counter  
    uint8_t remBits = 0;
    byte temp = 0;
    byte byte_temp[255*8];//max message lenght 255 bytes
    
    k = 0;
    //Convert bits to byte size
    for (int i = 0; i< bytelength ; i++)
    {
      for (register uint8_t t=128; t>0 ; t = t/2) {
        if (Buffer[i] & t) BitSequence[k++] = 0x01;
        else BitSequence[k++] = 0x00;
       }       
     }
     
     // stuff a 0 after five consecutive 1s.
     for (int i = 0; i < k ; i++)
     {
        if (BitSequence[i] == 0x01) cnt++;
        else cnt = 0; // restart count at 1
     
        BitSequenceStuffed[s++] = BitSequence[i]; // add the bit to the final sequence

        if (cnt == 5) // there are five consecutive bits of the same value
        {
            BitSequenceStuffed[s++] = 0x00; // stuff with a zero bit
            cnt = 0; // and reset cnt to zero
        }
      }
      
      _size = 0;
       //Recreate 0b01111110 (FLAG) in byte size
      for (int i=0; i < 64 ; i++)
      { 
         Buffer[_size++] = 0x00;
         for (int j=0; j < 6 ; j++) 
         {
           Buffer[_size++] = 0x01;
         }
         Buffer[_size++] = 0x00;
      }
              
      for (int i=0; i < s ; i++) Buffer[_size++] = BitSequenceStuffed[i];
      
      //Insert 0b01111110 (FLAG)
       Buffer[_size++] = 0x00;
       for (int j=0; j < 6 ; j++) 
       {
         Buffer[_size++] = 0x01;
       }
       Buffer[_size++] = 0x00;
            
      for (int i = 0; i< 255*8 ; i++) byte_temp[i] = 0x00;
      
      //NRZI encoding
      for (int i=0; i < _size ; i++) 
      {
         if (Buffer[i] == 0x00) 
         {
           byte_temp[i+1] = ! byte_temp[i];
         }
         else 
         {
           byte_temp[i+1] = byte_temp[i];
         }
      }

      //extrabits = (_size+1) % 8;
      if (((_size+1) % 8) > 0) remBits = 8 - ((_size+1) % 8);
      
      for (int i = (_size + 1) ; i < (_size + 1 + remBits ) ; i++)
      {
         byte_temp[i] = 0x01;
      }

      //Convert to bit after NRZI and added remaining bits to form byte array
      Index = 0;
      for (int i = 0; i < (_size + 1 + remBits); i = i + 8)
      {
        temp = 0;
        if  (byte_temp[i] == 0x01)   temp = temp + 0b10000000;
        if  (byte_temp[i+1] == 0x01) temp = temp + 0b01000000;
        if  (byte_temp[i+2] == 0x01) temp = temp + 0b00100000;
        if  (byte_temp[i+3] == 0x01) temp = temp + 0b00010000;
        if  (byte_temp[i+4] == 0x01) temp = temp + 0b00001000;
        if  (byte_temp[i+5] == 0x01) temp = temp + 0b00000100;
        if  (byte_temp[i+6] == 0x01) temp = temp + 0b00000010;
        if  (byte_temp[i+7] == 0x01) temp = temp + 0b00000001;
        finalSequence[Index++] = temp;
      }
}

boolean logicXOR(boolean a, boolean b)
{
  return (a||b) && !(a && b); 
}

unsigned int CRC_CCITT (byte *Buffer, uint8_t bytelength)
{
  uint8_t OutBit = 0;
  unsigned int XORMask = 0x0000;
  unsigned int SR = 0xFFFF;
  
  for (int i=0; i<bytelength ; i++)
  {
    for (uint8_t b = 128 ; b > 0 ; b = b/2) {
       
      OutBit = SR & 1 ? 1 : 0; //Bit shifted out of shift register
    
      SR = SR>>1; // Shift the register to the right and shift a zero in

      XORMask = logicXOR((Buffer[i] & b),OutBit) ? MSB_LSB_swap_16bit(CRC_POLYGEN) : 0x0000;

      SR = SR ^ XORMask;
    }
  }
  return  MSB_LSB_swap_16bit(~SR);  
}

byte MSB_LSB_swap_8bit(byte v)
{
  // swap odd and even bits
  v = ((v >> 1) & 0x55) | ((v & 0x55) << 1);
  // swap consecutive pairs
  v = ((v >> 2) & 0x33) | ((v & 0x33) << 2);
  // swap nibbles ... 
  v = ((v >> 4) & 0x0F) | ((v & 0x0F) << 4);
  return v;
}

unsigned int MSB_LSB_swap_16bit(unsigned int v)
{
  // swap odd and even bits
  v = ((v >> 1) & 0x5555) | ((v & 0x5555) << 1);
  // swap consecutive pairs
  v = ((v >> 2) & 0x3333) | ((v & 0x3333) << 2);
  // swap nibbles ... 
  v = ((v >> 4) & 0x0F0F) | ((v & 0x0F0F) << 4);
  // swap bytes
  v = ((v >> 8) & 0x00FF) | ((v & 0x00FF) << 8);  
  return v;
}
