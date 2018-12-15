/*
    Wireless Serial using UDP ESP8266
    Hardware: NodeMCU
    Master Board creates Access Point
*/
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>
#include <Cytron_PS2Shield.h>

#define LEDPIN  2

/*


Function:
  readButton(button); // Read button status, it will return corresponding data
                      // Digital button: 0 = pressed, 1 = released
                      // Analog button: return a value

  Digital button:
    PS2_SELECT
    PS2_JOYSTICK_LEFT
    PS2_JOYSTICK_RIGHT
    PS2_START
    PS2_UP
    PS2_RIGHT
    PS2_DOWN
    PS2_LEFT
    PS2_LEFT_2
    PS2_RIGHT_2
    PS2_LEFT_1
    PS2_RIGHT_1
    PS2_TRIANGLE
    PS2_CIRCLE
    PS2_CROSS
    PS2_SQUARE

  Analog button:
    PS2_JOYSTICK_LEFT_X_AXIS
    PS2_JOYSTICK_LEFT_Y_AXIS
    PS2_JOYSTICK_RIGHT_X_AXIS
    PS2_JOYSTICK_RIGHT_Y_AXIS
    PS2_JOYSTICK_LEFT_UP
    PS2_JOYSTICK_LEFT_DOWN
    PS2_JOYSTICK_LEFT_LEFT
    PS2_JOYSTICK_LEFT_RIGHT
    PS2_JOYSTICK_RIGHT_UP
    PS2_JOYSTICK_RIGHT_DOWN
    PS2_JOYSTICK_RIGHT_LEFT
    PS2_JOYSTICK_RIGHT_RIGHT

*/

Cytron_PS2Shield ps2(4,5); // SoftwareSerial: Rx and Tx pin 

const char *ssid = "controller";
const char *pass = "csdrobocon";
char ps2_dig;
bool ps2_data_dig[16];
char ps2_hexa_dig[5];

unsigned int localPort = 2000; // local port to listen for UDP packets

IPAddress ServerIP(192,168,4,1);
IPAddress ClientIP(192,168,4,2);

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

char packetBuffer[9]; 
//=======================================================================
//                Setup
//=======================================================================
void setup()
{
    Serial.begin(9600);
    ps2.begin(9600); // This baudrate must same with the jumper setting at PS2 shield
    Serial.println();
    WiFi.softAP(ssid, pass);    //Create Access point

    //Start UDP
    Serial.println("PROGRAM: starting UDP");
    if (udp.begin(localPort) == 1)
    {
        Serial.println("PROGRAM: UDP started");
        Serial.print("Local port: ");
        Serial.println(udp.localPort());
    }
    else
    {
      Serial.println("PROGRAM: UDP not started");
    }
}
//======================================================================
//                MAIN LOOP
//======================================================================
void loop()
{
    read_ps2();                                                         //read the play station digital pins and store them
    convert_to_hexa();                                                  //create a packet to send the data..i am using hexa characters
    int cb = udp.parsePacket();
 
    if (!cb) 
    {
        //if there is no data to read
        udp.beginPacket(ClientIP, 2000);
        //Send UDP requests to port 2000
        
        char a[4];
        a[0]=ps2_hexa_dig[0];
        a[1]=ps2_hexa_dig[1];
        a[2]=ps2_hexa_dig[2];
        a[3]=ps2_hexa_dig[3];
        //Serial.println(k);
        udp.write(a,4); //Send four bytes to ESP8266 
        udp.endPacket();
 
    }
    else {                                                              
      //if any UDP packet is received then display that on serial monitor,not really needed for the controller task
      udp.read(packetBuffer, 4); // read the packet into the buffer, we are reading only one byte
      Serial.print(packetBuffer);
    }
    delay(20);
}
//======================================================================
void read_ps2()
{
  ps2_data_dig[0]=ps2.readButton(PS2_SELECT);
  ps2_data_dig[1]=ps2.readButton(PS2_JOYSTICK_LEFT);
  ps2_data_dig[2]=ps2.readButton(PS2_JOYSTICK_RIGHT);
  ps2_data_dig[3]=ps2.readButton(PS2_START);
  ps2_data_dig[4]=ps2.readButton(PS2_UP);
  ps2_data_dig[5]=ps2.readButton(PS2_RIGHT);
  ps2_data_dig[6]=ps2.readButton(PS2_DOWN);
  ps2_data_dig[7]=ps2.readButton(PS2_LEFT);
  ps2_data_dig[8]=ps2.readButton(PS2_LEFT_2);
  ps2_data_dig[9]=ps2.readButton(PS2_RIGHT_2);
  ps2_data_dig[10]=ps2.readButton(PS2_LEFT_1);
  ps2_data_dig[11]=ps2.readButton(PS2_RIGHT_1);
  ps2_data_dig[12]=ps2.readButton(PS2_TRIANGLE);
  ps2_data_dig[13]=ps2.readButton(PS2_CIRCLE);
  ps2_data_dig[14]=ps2.readButton(PS2_CROSS);
  ps2_data_dig[15]=ps2.readButton(PS2_SQUARE);
  
 }
//========================================================================
//this is one way of packing the data,other methods can also be tried out

void convert_to_hexa()                                                    //converting 4bits to a hexa character 
 {
  int rec[4] = {0,0,0,0};
  for(int j=0;j<4;j++)
  {
    for(int i=0; i<4; i++)
    {
      rec[j] |= ps2_data_dig[j*4+i] << i;
    }  
    if(rec[j]>=0 && rec[j]<=9)                                            
    ps2_hexa_dig[j]='0'+(rec[j]);                                             
    else
    ps2_hexa_dig[j]='A'+(rec[j]-10);
  }
  }

 
