
byte s1_for =0;
byte s1_rew =1;
byte s2_for =2;
byte s2_rew =3;

byte s1_spd =5;
byte s2_spd =6;
int spd = 255;


// the setup routine runs once when you press reset:
void setup()  { 
  pinMode(s1_for, OUTPUT);
  pinMode(s1_rew, OUTPUT);
  pinMode(s2_for, OUTPUT);
  pinMode(s2_rew, OUTPUT);
  
  digitalWrite(s1_for, LOW);
  digitalWrite(s1_rew, LOW);
  digitalWrite(s2_for, LOW);
  digitalWrite(s2_rew, LOW);
  
  analogWrite(s1_spd, spd);
  analogWrite(s2_spd, spd);
} 

// the loop routine runs over and over again forever:
void loop()  { 
  // reverse the direction of the fading at the ends of the fade: 
  digitalWrite(s1_for, HIGH);
  delay(1000);
  for (int licz = 0 ;licz < 260;licz = licz + 5) 
   {analogWrite(s1_spd, licz);
   delay(100);
   } 
  digitalWrite(s1_for, LOW);
   
  digitalWrite(s1_rew, HIGH);
  delay(1000);
  for (int licz = 0 ;licz < 260;licz = licz + 5) 
   {analogWrite(s1_spd, licz);
   delay(100);
   } 
  digitalWrite(s1_rew, LOW);

  digitalWrite(s2_for, HIGH);
  delay(1000);
  for (int licz = 0 ;licz < 260;licz = licz + 5) 
   {analogWrite(s2_spd, licz);
   delay(100);
   } 
  digitalWrite(s2_for, LOW);
   
  digitalWrite(s2_rew, HIGH);
  delay(1000);
  for (int licz = 0 ;licz < 260;licz = licz + 5) 
   {analogWrite(s2_spd, licz);
   delay(100);
   } 
  digitalWrite(s2_rew, LOW);
  
  
  digitalWrite(s2_rew, HIGH);
  digitalWrite(s1_rew, HIGH);
  delay(1000);
  for (int licz = 0 ;licz < 260;licz = licz + 5) 
   {analogWrite(s2_spd, licz);
    analogWrite(s1_spd, 255-licz);
   delay(100);
   } 
  digitalWrite(s2_rew, LOW);
  digitalWrite(s1_rew, LOW);
  
  digitalWrite(s2_for, HIGH);
  digitalWrite(s1_for, HIGH);
  delay(1000);
  for (int licz = 0 ;licz < 260;licz = licz + 5) 
   {analogWrite(s2_spd, licz);
    analogWrite(s1_spd, licz);
   delay(100);
   } 
  for (int licz = 0 ;licz < 260;licz = licz + 5) 
   {analogWrite(s2_spd, 255-licz);
    analogWrite(s1_spd, 255-licz);
   delay(100);
   } 
   
  digitalWrite(s2_for, LOW);
  digitalWrite(s1_for, LOW);
  
  
}


