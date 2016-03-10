#ifndef _AVERAGE_H_
#define _AVERAGE_H_

class Average {
  public:
    Average(unsigned char Size);
    unsigned char Rolling_Average(unsigned char Value);
  
  private:
    unsigned char Num_Buffer[10];
    unsigned char Index;
    unsigned char FILTER_SIZE;
};

Average::Average(unsigned char Size)
{
  this->FILTER_SIZE = Size;
  this->Index = 0; 

  unsigned char i;
  for(i = 0;i < FILTER_SIZE;i++)
    this->Num_Buffer[i] = 0;
}

unsigned char Average::Rolling_Average(unsigned char Value)
{
  this->Num_Buffer[this->Index] = Value;                                // Put new value into buffer array.
  this->Index++;
    
  if(this->Index == FILTER_SIZE)                                        // Roll back Index if it overflows
    this->Index = 0;

  unsigned char i;
  unsigned int sum = 0;
  for(i = 0;i < FILTER_SIZE;i++)
    sum += this->Num_Buffer[i];

  return(sum/FILTER_SIZE);
}

#endif