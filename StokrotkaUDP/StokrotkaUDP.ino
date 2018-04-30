

/*
  UDPSendReceiveString:
  This sketch receives UDP message strings, prints them to the serial port
  and sends an "acknowledge" string back to the sender

  A Processing sketch is included at the end of file that can be used to send
  and received messages for testing with a computer.

  created 21 Aug 2010
  by Michael Margolis

  This code is in the public domain.
*/
#include <SD.h>
#define SD_ChipSelectPin 4  //using digital pin 4 on arduino nano 328, can use other pins
#include <TMRpcm.h>           //  also need to include this library...
#include <SPI.h>


#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008



// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

IPAddress dnServer(192, 168, 0, 1);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress ip(192, 168, 0, 222);


unsigned int localPort = 8888;              // local port to listen on

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet,
char  ReplyBuffer[] = "Czesc Piter";       // a string to send back
String inputString = "";                    // a String to hold incoming data
boolean stringComplete = false;             // whether the string is complete


EthernetUDP Udp; // An EthernetUDP instance to let us send and receive packets over UDP
TMRpcm tmrpcm;   // create an object for use in this sketch


void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  // setup SD-card
  Serial.print("Initializing SD card...");
  if (!SD.begin(4)) {
    Serial.println(" failed!");
    while (true);
  }
  
  Serial.println(" done.");
  // hi-speed SPI transfers
  SPI.setClockDivider(4);
  
  // start the Ethernet and UDP:
  //Ethernet.begin(mac, ip, dnServer, gateway, subnet);
  Ethernet.begin(mac);
  
  Serial.print("IP1 = ");
  Serial.println(Ethernet.localIP());

  inputString.reserve(20);

  Udp.begin(localPort);
  
  pinMode(LED_BUILTIN, OUTPUT);

  tmrpcm.speakerPin = 46; //5,6,11 or 46 on Mega, 9 on Uno, Nano, etc
  pinMode(45,OUTPUT);// Pin pairs: 9,10 Mega: 5-2,6-7,11-12,46-45 //Complimentary Output or Dual Speakers:
  
}

void loop() {
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From :");
    Serial.print(Udp.remoteIP());
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    //packetBuffer=0;
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    Serial.println("Contents:");
    Serial.println(packetBuffer);

    // send a reply to the IP address and port that sent us the packet we received
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(ReplyBuffer);
    Udp.endPacket();
    
    if (packetBuffer[0] == 'R') {
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.print("Gram: ");
      //Udp.stop();
      play_audio(packetBuffer);
      //Udp.begin(localPort);
      Serial.println(" END");
    };
    
    if (packetBuffer[2] == 0x31) {
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.println("Kasa 1");
    };

    if (packetBuffer[2] == 0x32) {
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("Kasa 2");
      play_audio(packetBuffer);
    };

  }
  delay(10);

  if (stringComplete) {
    Serial.println(inputString);
    obsluga_serial();
    inputString = "";
    stringComplete = false;
  }
}

void obsluga_serial() {
  if ( inputString == "config") {

    konfiguruj_modul();

  }
  if ( inputString == "haslo") {
    Serial.println("odzew");
  }

  if ( inputString == "stop") {
    Serial.println("stop");
    tmrpcm.stopPlayback();
  }

}


void konfiguruj_modul( )
{
  Serial.println("");
  Serial.println("******************");
  Serial.println("*  Konfiguracja  *");
  Serial.println("******************");
  delay(1000);
  Serial.println("");
  Serial.println("***********");
  Serial.println("*  Zapis  *");
  Serial.println("***********");

}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    if ((inChar == '\n') or (inChar == '\r')) {
      stringComplete = true;
    }
    else {
      inputString += inChar;
    }
  }
}


void play_audio(String nazwa) {
  tmrpcm.play("music");
  delay(10000);
  tmrpcm.stopPlayback();
}

