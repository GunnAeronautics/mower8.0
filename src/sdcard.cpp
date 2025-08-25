
#include <sdcard.h>
String logFilename;
String datalogHeader;
volatile int howManyDatas = 0;
bool lastLED = true;
const int chipSelect = 10;

String getNewLogFilename(){
  int fileIndex = 1;
  String filename;
  do
  {
    filename = "/datalog" + String(fileIndex) + ".csv";
    fileIndex++;
  } while (SD.exists(filename.c_str()));
  return filename;
}
// appendFile(SD, "/hello.txt", "World!\n");
void sdSetup()
{
  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
    while (true);
  }

  Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open(getNewLogFilename());

  // if the file is available, write to it:
  if (dataFile) {
    while (dataFile.available()) {
      Serial.write(dataFile.read());
    }
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}


