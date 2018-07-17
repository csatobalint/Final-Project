signed int x;
signed int y;
signed int z;
signed int w;

float a;
float m;

String str;
char dir = 0;
int start_val = 0;

int Mf = 2;  
int Ml = 4; 
int Mb = 7;

int Ef = 3; 
int El = 9; 
int Eb = 10; 

int e1 = 0;
int e2 = 0;
int e3 = 0;

int e1p = 0;
int e2p = 0;
int e3p = 0;

int kicker = 0;

int i = 0;
int counter = 0;

boolean fast = true;
boolean StateA0 = false;
boolean StateA1 = false;
boolean State11 = false;

char getData;

double set_speed = 1;
double speed_val = 1;

void process();
void turnright();
void turnleft();

void setup() {
  
pinMode(Mf, OUTPUT);    //direction of forward motor (located on upper right position)
pinMode(Ml, OUTPUT);    //direction of left motor (located on upper left position)
pinMode(Mb, OUTPUT);    //direction of back motor (located on bottom position)
pinMode(Ef, OUTPUT);    //speed of forward motor
pinMode(El, OUTPUT);    //speed of left motor
pinMode(Eb, OUTPUT);    //speed of back motor
pinMode(A0, INPUT);
pinMode(A1, INPUT);
pinMode(6, OUTPUT);
pinMode(11, OUTPUT);
pinMode(12, OUTPUT);
pinMode(13, OUTPUT);

  Serial.begin(9600);
  Serial.println("Start");
  
  for(i = 0; i < 10; i++) {
    speed_val = analogRead(A0);
    speed_val = speed_val / 1023 * 5;
  }
  
  digitalWrite(6, LOW); 
}

void test()  // for communication testing purpose
{
  if (digitalRead(A1) == LOW) 
  {
    counter++;
    if (counter > 10) counter = 0;
    digitalWrite(12, HIGH);
    Serial.println("Success (" + String(counter) + ")");
    delay(500);
  }
  else digitalWrite(12, LOW);  
}

void loop() 
{
  test();  //test communication (optional)
    
    while(Serial.available())
    {
      
     test();  //test communication (optional)
     
      char getData = Serial.read();
      
        if (getData == 'M')
        {
          m = Serial.parseFloat();
          
          if (Serial.read() == '#') 
          {
            processM();
          }
        }
        
        if (getData == 'A')
        {
          a = Serial.parseFloat();
          
          if (Serial.read() == '#') 
          {
            processA();
          }
        }
       
        if (getData == 'a')
        {  
          delay(5);        
          if (Serial.read() == '#') 
          {
            turnright();
          }
        }      
       
        if (getData == 'b')
        {    
          delay(5);
          if (Serial.read() == '#') 
          {
            stoprobot();
          }
        }   

        if (getData == 'c')
        {     
          delay(5);
          if (Serial.read() == '#') 
          {
            turnleft();
          }
        } 

        if (getData == 'x')
        {  
          delay(5);        
          if (Serial.read() == '#') 
          {
            processx();
          }
        }      
       
        if (getData == 'y')
        {    
          delay(5);
          if (Serial.read() == '#') 
          {
            processy();
          }
        }   

        if (getData == 'z')
        {     
          delay(5);
          if (Serial.read() == '#') 
          {
            processz();
          }
        } 
        
        if (getData == '~') {
        
          x = Serial.parseInt();
        
            if (Serial.read() == '*') {
        
              y = Serial.parseInt();
        
                if (Serial.read() == '@') {
                
                  z = Serial.parseInt();
                  
                    if (Serial.read() == '.') {
                      
                      w = Serial.parseInt();
                      
                        if (Serial.read() == '#') {
                          
                          process();
                        }
                  }                
              } 
           }
        }
        
          
        
    }
    
}

void processa(){              //button a pressed
  Serial.println("Button a! ");
  StateA0 = !StateA0;
  if (StateA0 == true) { digitalWrite(A0, HIGH); } else { digitalWrite(A0, LOW); }
  Serial.flush();  
}

void processb(){              //button b pressed
  Serial.println("Button b! "); 
  State11 = !State11;
  if (State11 == true) { digitalWrite(11, HIGH); } else { digitalWrite(11, LOW); }
  Serial.flush(); 
}

void processc(){              //button c pressed
  Serial.println("Button c! ");
  StateA1 = !StateA1;
  if (StateA1 == true) { digitalWrite(A1, HIGH); } else { digitalWrite(A1, LOW); }
  Serial.flush();  
}

void processx(){              //button x pressed
  Serial.println("Button x! "); 
  Serial.flush(); 
  fast = true;
}

void processy(){              //button y pressed
  Serial.println("Button y! "); 
  Serial.flush(); 
}

void processz(){              //button z pressed
  Serial.println("Button z! "); 
  Serial.flush(); 
  fast = false;
}

void processM(){              //button Magnitude pressed
  Serial.print("Received Magnitude: ");
  Serial.println(m); 
  Serial.flush();   
}

void processA(){              //button Angle pressed
  Serial.print("Received Angle: ");
  Serial.println(a); 
  Serial.flush();   
}

void turnright() {
  
  Serial.println("TURNING RIGHT");
  
  digitalWrite(Ml, HIGH);
  digitalWrite(Mf, HIGH);
  digitalWrite(Mb, HIGH);

  speed_val = analogRead(A0);
  speed_val = speed_val / 1023 * 5;
  
  e1p = 100 * speed_val;
  e2p = 100 * speed_val;  
  e3p = 100 * speed_val;
  
  analogWrite(El, e2p);  
  analogWrite(Eb, e3p); 
  analogWrite(Ef, e1p); 
}

void turnleft() {
  
  Serial.println("TURNING LEFT");
  
  digitalWrite(Ml, LOW);
  digitalWrite(Mf, LOW);
  digitalWrite(Mb, LOW);

  speed_val = analogRead(A0);
  speed_val = speed_val / 1023 * 5;
  
  e1p = 100 * speed_val;
  e2p = 100 * speed_val;  
  e3p = 100 * speed_val;
  
  analogWrite(El, e2p);  
  analogWrite(Eb, e3p); 
  analogWrite(Ef, e1p); 
}
  
void stoprobot() {
  
  Serial.println("STOP");
  
  digitalWrite(Ml, LOW);
  digitalWrite(Mf, LOW);
  digitalWrite(Mb, LOW);

  speed_val = analogRead(A0);
  speed_val = speed_val / 1023 * 5;
  
  e1p = 0 * speed_val;
  e2p = 0 * speed_val;  
  e3p = 0 * speed_val;
  
  analogWrite(El, e2p);  
  analogWrite(Eb, e3p); 
  analogWrite(Ef, e1p); 
}

void process() {

  speed_val = analogRead(A0);           //robot's overall speed control
  speed_val = speed_val / 1023 * 5;
      
//  Serial.print("Received x: ");
//  Serial.print(x);              
//  Serial.print("      Received y: ");
//  Serial.print(y);       
//  Serial.print("         Received z: ");
//  Serial.print(z);       
//  Serial.print("            Received w: ");
//  Serial.println(w); 
  
  kicker = w;
  
  if (kicker == 1)                     //switch on kicking mechanism 
  {
    digitalWrite(6, HIGH); 
    digitalWrite(11, HIGH);
    digitalWrite(13, HIGH);    
    delay(1); 
  }
  else if (kicker == 0)                //switch off kicking mechanism 
  {
    digitalWrite(6, LOW); 
    digitalWrite(11, LOW); 
    digitalWrite(13, LOW);  
  }
  else if (kicker == 2)                //switch on and off kicking mechanism 
  {
    digitalWrite(6, HIGH);
    digitalWrite(11, HIGH);
    digitalWrite(13, HIGH);       
    delay(200); 
    digitalWrite(6, LOW); 
    digitalWrite(11, LOW);
    digitalWrite(13, LOW);      
    delay(500);     
  }
  
  e1 = map(x, 0, 255, 0, 255);
//  Serial.print("Motor 1: ");
///  Serial.print(e1); 
  if (e1 < 0) { digitalWrite(Mf, LOW); //Serial.print("  M1 Low"); 
  } 
  else if (e1 >= 0)  { digitalWrite(Mf, HIGH); //Serial.print("  M1 High"); 
  } 
  e1p = abs(e1);

  
  e2 = map(y, 0, 255, 0, 255);
//  Serial.print("Motor 2: ");
///  Serial.print(e2); 
  if (e2 < 0) { digitalWrite(Ml, LOW); //Serial.print("  M2 Low"); 
  } 
  else if (e2 >= 0) { digitalWrite(Ml, HIGH); //Serial.print("  M2 High"); 
  } 
  e2p = abs(e2);

  
  e3 = map(z, 0, 255, 0, 255);
//  Serial.print("Motor 3: ");
///  Serial.print(e3); 
  if (e3 < 0) { digitalWrite(Mb, LOW); //Serial.println("  M3 Low"); 
  } 
  else if (e3 >= 0)  { digitalWrite(Mb, HIGH); //Serial.println("  M3 High"); 
  } 
  
  e3p = abs(e3);
  
  e1p = e1p * speed_val;
  e2p = e2p * speed_val;  
  e3p = e3p * speed_val;
  
  analogWrite(El, e2p);  
  analogWrite(Eb, e3p); 
  analogWrite(Ef, e1p);    
  
  
  Serial.flush(); 
}  
