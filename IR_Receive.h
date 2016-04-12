#ifndef _IR_RECEIVE_H_
#define _IR_RECEIVE_H_

/* This is the IR_Receive class. Before IR Receiving works the user must intialise a few things outlined in the minimal Rx example code ("IR_Rx_lib") 
 *  
 * The Expected Bits can be configured by calling Set_Expected_Bits(char bits) etc. The default is 17. The rx will work if more bits are transmitted
 * but will fail should more bits be expected than are actually received.
 *  
 * Since Transmissions are expected every 40ms, There's no point listening for the start of the new packet until just before the packet is due. That's 
 * why the IR_POST_RX_SILENT_PERIOD exists. The function call No_Post_Rx_Silence() sets this silent period to zero which may be useful should irregular
 * Transmissions be used or a faster data rate attempted. I haven't tried using it though. 
 * 
*/

//--------------------------------------------------------------------------------
class IR_Receive {
  public:
    IR_Receive();
    long Check_Data();                                                            // Checks if data is available and returns it if so.
    unsigned long Packet_Start_Time();                                            // Returns the micros() start time of the last successfully received packet.
    void Switch_Channel(char new_channel);                                        // Switch channel 1 or 2.
    
    void Timer_Interrupt();
    void Pin_Change_Interrupt();
    
    void Set_Expected_Bits(char New_Expected_Bits);
    void No_Post_Rx_Silence();
    void Configure_Timer2_Interrupts();
  
  private:  
    volatile unsigned long  Total_Time_High;                                      // Measures the time spent high during a bit_period. If Total_Time_High > 1/2 * Bit period we assume a 1 was sent. Filters spikes
    volatile unsigned long  TimeA, Time_Diff;
    volatile unsigned long  Encoded_Data[2];
    volatile unsigned long  Time_at_Packet_Start;
    
    volatile boolean        Decode_Flag;
    volatile boolean        State;
    volatile char           bit_index = 0;

    unsigned int            IR_POST_RX_SILENT_PERIOD = 24000;                     // Time to not listen for after a receive: Transmission period (40ms) - (packet length) - leeway time (1.6ms)
    unsigned int            IR_PACKET_LENGTH = 14400;                             // Including start bit.
    char                    EXPECTED_BITS = 17;                                   // For IR receiving. The number of bits we expect to receive (not including the start bit)
    char                    IR_Channel = 1;                                       // Default Channel to listen on.
};

IR_Receive::IR_Receive()
{
}

//--------------------------------------------------------------------------------
void IR_Receive::Configure_Timer2_Interrupts()
{
  //Configure Timer2 to interrupt every 0.4ms for IR receiving: 
  TCCR2A = 0;  
  TCCR2B = 0;  
  TCNT2  = 0;  
  OCR2A  = 49;                                                                    // Interrupt every 0.4ms for IR receiving
  TCCR2A = 0b00000010;                                                            // Clear Timer on Compare match mode
  TCCR2B = 0b00000101;                                                            // ticks at 125KHz  
  TIMSK2 = 0b00000000;                                                            // Compare match on OCR2A interrupt (set this once signal is being received) TIMSK2 = 0b00000010;
}

//--------------------------------------------------------------------------------
void IR_Receive::Set_Expected_Bits(char New_Expected_Bits)
{   
  EXPECTED_BITS = New_Expected_Bits;
  IR_PACKET_LENGTH = (800 * EXPECTED_BITS + 1);
  IR_POST_RX_SILENT_PERIOD = 40000 - IR_PACKET_LENGTH - 1600;
}

//--------------------------------------------------------------------------------
void IR_Receive::No_Post_Rx_Silence()
{   
  IR_POST_RX_SILENT_PERIOD = 0;
}

//--------------------------------------------------------------------------------
unsigned long IR_Receive::Packet_Start_Time()
{
  return(Time_at_Packet_Start);
}

//--------------------------------------------------------------------------------
void IR_Receive::Switch_Channel(char new_channel)
{
  IR_Channel = new_channel;
}

//--------------------------------------------------------------------------------
long IR_Receive::Check_Data()                                                     // Encoded_Data from ISR is processed to a long. If data is erronous or not present returns 0
{
  if(!Decode_Flag)
    return(0);    
                                                                            
  char i, index; 
  char BitH;
  char BitL;
  unsigned long IR_Data = 0;
  unsigned long Encoded_Data_Safe[2];
  
  Encoded_Data_Safe[0] = Encoded_Data[0];
  Encoded_Data_Safe[1] = Encoded_Data[1];
  Decode_Flag = 0;

  index = (EXPECTED_BITS - 1);
  
  if(EXPECTED_BITS > 16)  {    
    for(i = (((EXPECTED_BITS - 16) * 2) - 1); i >= 0;i -= 2)
    {
      BitH = bitRead(Encoded_Data_Safe[1], i);
      BitL = bitRead(Encoded_Data_Safe[1], i-1);
  
      if(BitH && !BitL)
        bitSet(IR_Data, index); 
      else if(!BitH && BitL)
        bitClear(IR_Data, index); 
      else
        return(0);                                                                // Erronous data, ignore packet
  
      index--;
    } 
    
    for(i = 31; i >= 0;i -= 2)
    {
      BitH = bitRead(Encoded_Data_Safe[0], i);
      BitL = bitRead(Encoded_Data_Safe[0], i-1);
  
      if(BitH && !BitL)
        bitSet(IR_Data, index); 
      else if(!BitH && BitL)
        bitClear(IR_Data, index); 
      else
        return(0);                                                                // Erronous data, ignore packet
  
      index--;
    }       
  }
  else  {
    for(i = ((EXPECTED_BITS * 2) - 1); i >= 0;i -= 2)
    {
      BitH = bitRead(Encoded_Data_Safe[0], i);
      BitL = bitRead(Encoded_Data_Safe[0], i-1);
  
      if(BitH && !BitL)
        bitSet(IR_Data, index); 
      else if(!BitH && BitL)
        bitClear(IR_Data, index); 
      else
        return(0);                                                                // Erronous data, ignore packet
  
      index--;
    }     
  }
  
  return(IR_Data);
}

//--------------------------------------------------------------------------------
void IR_Receive::Pin_Change_Interrupt()
{
    Time_Diff = micros() - TimeA;                                                  
    State = IR_IN_STATE;
  
    if(bit_index)                                                                 // If signal has started
    {    
      TimeA = Time_Diff + TimeA;                                                  // TimeA = micros() esentially, the time at beginning of this interrupt
      
      if(State == LOW)                                                            //Signal must have just gone low so Time_Diff holds the most recent high pulse width
      {
        Total_Time_High += Time_Diff;
        return;
      }    
    } 
  
    // Here we detect whether the the signal has just started. If it has, set bit_index to EXPECTED_BITS*2 + 1. We only detect start bits a set nummber of microseconds after 
    // the last packet finishes (i.e just befor next pulse is due to arrive). This way we should sync with the transmitter and improve our robustness to interference etc.
    if (!State && (Time_Diff > IR_POST_RX_SILENT_PERIOD))
    {   
      TimeA = Time_Diff + TimeA;                                                  // TimeA = micros() esentially, the time at beginning of this interrupt
      
      TCNT2 = 0;                                                                  // Reset the counter.
      OCR2A = 99;                                                                 // Interrupt in 800us - Start bit measurement
      
      TIFR2 = 0b00000010;                                                         // Reset any counter interrupt 
      TIMSK2 = 0b00000010;                                                        // Enable interrupts on OCR2A compare match          
      bit_index = (EXPECTED_BITS * 2) + 1;                                          
  
      return;        
    }  
}

//--------------------------------------------------------------------------------
// Once the start of a packet is detected, this routine works out if a 1 or 0 was sent after each bit period.
void IR_Receive::Timer_Interrupt()
{  
  static unsigned long DataH, DataL;
  
  if(bit_index)                                                                   // If a receive has started IF TIMER2 ONLY GETS STARTED AFTER A START PULSE IS THIS NEEDED?
  {      
    bit_index--;
    
    Time_Diff = micros() - TimeA;
    TimeA = Time_Diff + TimeA;                                                    // This is faster than TimeA = micros() and more accurate

    if(State == HIGH) {
      Total_Time_High += Time_Diff;  
    }

    if(bit_index == (EXPECTED_BITS * 2))                                          // If this is the first bit... 
    {
      if(IR_Channel == 1)  {                                                           
        if(Total_Time_High > 475 && Total_Time_High < 625)  {                     // Channel 1 start bits are ~600us long. COULD NARROW THESE WONDOWS DOWN SOMEWHAT.
          Total_Time_High = 0;
          OCR2A = 49;                                                             // Interrupt every 400us from now until the end of the packet. 
          return;
        }
      }
      else  {
        if (Total_Time_High > 175 && Total_Time_High < 325)  {                    // Channel 2 start bits are ~250us long. COULD NARROW THESE WONDOWS DOWN SOMEWHAT.
          Total_Time_High = 0;
          OCR2A = 49;                                                             // Interrupt every 400us from now until the end of the packet.
          return;
        }
      }
      
      bit_index = 0;                                                              // Cancel this receive and search for start bit again.
      Total_Time_High = 0;
      TIMSK2 = 0b00000000;                                                        // Disable OCR2A interrupts      
      TimeA -= IR_POST_RX_SILENT_PERIOD;                                          // Immediately start searching for start bits again.
      return;      
    }    

    if(bit_index > 31)  {
      if(Total_Time_High > 200) {
        bitWrite(DataH, (bit_index - 32), 1);           
      }
      else  {
        bitWrite(DataH, (bit_index - 32), 0); 
      }        
    }
    else  {
      if(Total_Time_High > 200) {
        bitWrite(DataL, bit_index, 1);           
      }
      else  {
        bitWrite(DataL, bit_index, 0); 
      }  
    } 
    
    Total_Time_High = 0;

    if(!bit_index)  {
      TIMSK2 = 0b00000000;                                                        // Disable OCR2A interrupts
      
      Decode_Flag = 1;
      Encoded_Data[0] = DataL; 
      Encoded_Data[1] = DataH;

      Time_at_Packet_Start = TimeA - IR_PACKET_LENGTH;
    }
  }
}


#endif











