/*=========================================
 SWARM ROBOTS ISP DEBUG PROGRAM
 Rayce Stipanovich
 =========================================*/

//Include Libraries and Packages
import controlP5.*;
import org.gwoptics.graphics.*;
import org.gwoptics.graphics.graph2D.*;
import org.gwoptics.graphics.graph2D.Graph2D;
import org.gwoptics.graphics.graph2D.LabelPos;
import org.gwoptics.graphics.graph2D.traces.Line2DTrace;
import org.gwoptics.graphics.graph2D.traces.ILine2DEquation;
import org.gwoptics.graphics.graph2D.traces.RollingLine2DTrace;
import org.gwoptics.graphics.graph2D.backgrounds.*;
import processing.serial.*;
import java.util.Calendar;
import java.text.SimpleDateFormat;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

/*=========================================
 DEMO MODE
 =========================================*/
public static final boolean demoMode = false;

/*=========================================
 Global Defines
 =========================================*/
//Serial
int baudrate = 9600;
Serial comPort;
int serialPortNumber = -1;
boolean foundSerial = false;
boolean autoConnectSerial = true;
String currentSerialDevice = "";
byte previousByte = 0;
byte lastByte = 0;
byte inByte = 0;
int bytesToRead = 35;
byte[] serialBuffer = new byte[bytesToRead];
int placeInFrame = 0;
int droppedPackets = 0;
long packetNumber = 1;
boolean inPacket = false;
long serialBtyesRec = 0;

//GraphingS
RollingLine2DTrace r1, r2, r3, r4, r5, r6;
Graph2D g1, g2, g3, g4, g5;
GridBackground gb1, gb2, gb3, gb4, gb5;

//UI
ControlP5 cp5;
Textarea consoleTextarea;
DropdownList portChooser;
CheckBox checkbox;
PFont f;
Textfield sendDialog;

//Scheduling
ScheduledExecutorService worker = Executors.newSingleThreadScheduledExecutor();
public static final String DATE_FORMAT_NOW = "yyyy-MM-dd HH:mm:ss";

/*=========================================
 Defines for Data
 =========================================*/
int IR0, IR1, IR2, IR3, IR4, IR5, IR6, IR7;//
int MEMS0, MEMS1, MEMS2, MEMS3;            //
int LDR;                                   //graphed
boolean ES0, ES1, ES2, ES3;                //
int IMU_X, IMU_Y, IMU_Z;                   //
int TEMP, CURR, RSSI, VOL0, VOL1;          //graphed
boolean BS0, BS1, BS2, BS3;                //
boolean DP0, DP1, DP2, DP3;                //

//define input ranges
public static final float raw_ir_min      = 1200.0;
public static final float raw_ir_max      = 4500.0;
public static final float raw_mems_min    = 0.0;
public static final float raw_mems_max    = 6000.0;
public static final float raw_ldr_min     = 0.0;
public static final float raw_ldr_max     = 6000.0;
public static final float raw_temp_min    = 0.0;
public static final float raw_temp_max    = 6000.0;
public static final float raw_rssi_min    = 0.0;
public static final float raw_rssi_max    = 6000.0;
public static final float raw_cur_min     = 0.0;
public static final float raw_cur_max     = 6000.0;
public static final float raw_vol_min     = 0.0;
public static final float raw_vol_max     = 7600.0;

//define scaled output ranges for graphs
public static final float prt_ldr_min     = 0.0;
public static final float prt_ldr_max     = 100.0;
public static final float prt_temp_min     = 0.0;
public static final float prt_temp_max     = 140.0;
public static final float prt_cur_min     = 0.0;
public static final float prt_cur_max     = 200.0;
public static final float prt_rssi_min     = 0.0;
public static final float prt_rssi_max     = 100.0;
public static final float prt_ir_min     = 0.0;
public static final float prt_ir_max     = 130.0;
public static final float prt_mems_min     = 0.0;
public static final float prt_mems_max     = 130.0;
public static final float prt_vol_min      = 2.7;
public static final float prt_vol_max      = 4.3;

public void initializeData(){
  IR0 = IR1 = IR2 = IR3 = IR4 = IR5 = IR6 = IR7 = 0;
  MEMS0 = MEMS1 = MEMS2 = MEMS3 = 0;
  LDR = TEMP = CURR = RSSI = VOL0 = VOL1 = 0;
  ES0 = ES1 = ES2 = ES3 = false;
  IMU_X = IMU_Y = IMU_Z = 0;
  BS0 = BS1 = BS2 = BS3 = false;
  DP0 = DP1 = DP2 = DP3 = false;
}

/*=========================================
 Functions for Console Logging
 =========================================*/
public static String now() {
  Calendar cal = Calendar.getInstance();
  SimpleDateFormat sdf = new SimpleDateFormat(DATE_FORMAT_NOW);
  return sdf.format(cal.getTime());
}

void logConsole(String toadd) {
  String oldtest;
  String date = now();
  oldtest = consoleTextarea.getText();
  if (oldtest != null && oldtest.length() >0) consoleTextarea.setText(oldtest + "  \n\r CONSOLE:  " + date + ":   " + toadd);
  else consoleTextarea.setText("  CONSOLE:  " + date + ":   " + toadd);
  consoleTextarea.scroll(1.0);
}

void logError(String toadd) {
  String oldtest;
  String date = now();
  oldtest = consoleTextarea.getText();
  if (oldtest != null && oldtest.length() >0) consoleTextarea.setText(oldtest + "  \n\r== ERROR:  " + date + ":   " + toadd);
  else consoleTextarea.setText("==  ERROR:  " + date + ":   " + toadd);
  consoleTextarea.scroll(1.0);
}

/*=========================================
 Graphing Functions
 =========================================*/
class eq1 implements ILine2DEquation{
  public double computePoint(double x,int pos) {
    if (demoMode) return 50+40*Math.sin(x);
    else return (double)map((float)LDR, raw_ldr_min, raw_ldr_max, prt_ldr_min, prt_ldr_max);
  }    
}
class eq2 implements ILine2DEquation{
  public double computePoint(double x,int pos) {
    if (demoMode) return 70+50*Math.sin(x);
    else return (int)map((float)TEMP, raw_temp_min, raw_temp_max, prt_temp_min, prt_temp_max);
  }    
}
class eq3 implements ILine2DEquation{
  public double computePoint(double x,int pos) {
    if (demoMode) return 50+50*Math.cos(x);
    else return (int)map((float)RSSI, raw_rssi_min, raw_rssi_max, prt_rssi_min, prt_rssi_max);
  }    
}
class eq4 implements ILine2DEquation{
  public double computePoint(double x,int pos) {
    if (demoMode) return 3.4+.6*Math.sin(x);
    else return (int)map((float)VOL0, raw_vol_min, raw_vol_max, prt_vol_min, prt_vol_max);
  }    
}
class eq5 implements ILine2DEquation{
  public double computePoint(double x,int pos) {
    if (demoMode) return 3.4+.6*Math.cos(x*2);
    else return (int)map((float)VOL1, raw_vol_min, raw_vol_max, prt_vol_min, prt_vol_max);
  }    
}
class eq6 implements ILine2DEquation{
  public double computePoint(double x,int pos) {
    if (demoMode) return 100+90*Math.cos(x);
    else return (int)map((float)CURR, raw_cur_min, raw_cur_max, prt_cur_min, prt_cur_max);
  }    
}

void setupGraphs() {
  r1  = new RollingLine2DTrace(new eq1(), 100, 0.1f);
  r1.setTraceColour(0, 100, 255);
  r2  = new RollingLine2DTrace(new eq2(), 100, 0.1f);
  r2.setTraceColour(255, 100, 0);
  r3  = new RollingLine2DTrace(new eq3(), 100, 0.1f);
  r3.setTraceColour(10, 255, 20);
  r4  = new RollingLine2DTrace(new eq4(), 100, 0.1f);
  r4.setTraceColour(245, 151, 0);
  r5  = new RollingLine2DTrace(new eq5(), 100, 0.1f);
  r5.setTraceColour(0, 245, 230);
  r6  = new RollingLine2DTrace(new eq6(), 100, 0.1f);
  r6.setTraceColour(255, 255, 10);;

  gb1 = new GridBackground(new GWColour(0));
  gb1.setGridColour(20,20,20,20,20,20);
  gb2 = new GridBackground(new GWColour(0));
  gb2.setGridColour(20,20,20,20,20,20);
  gb3 = new GridBackground(new GWColour(0));
  gb3.setGridColour(20,20,20,20,20,20);
  gb4 = new GridBackground(new GWColour(0));
  gb4.setGridColour(20,20,20,20,20,20);
  gb5 = new GridBackground(new GWColour(0));
  gb5.setGridColour(20,20,20,20,20,20);
  
  g1 = new Graph2D(this, 320, 80, false);
  g1.setYAxisMax(prt_ldr_max);
  g1.setYAxisMin(prt_ldr_min);
  g1.addTrace(r1);
  g1.position.y = 40;
  g1.position.x = 870;
  g1.setYAxisTickSpacing(20);
  g1.setXAxisMax(5f);
  g1.setNoBorder();
  g1.setBackground(gb1);
  g1.setAxisColour(90, 90, 90);
  g1.setFontColour(90, 90, 90);
  g1.setXAxisTickSpacing(5.0);
  g1.setXAxisMinorTicks(5);
  g1.setXAxisLabel("");
  g1.setYAxisLabel("Brightness %");
  
  g2 = new Graph2D(this, 320, 80, false);
  g2.setYAxisMax(prt_temp_max);
  g2.setYAxisMin(prt_temp_min);
  g2.addTrace(r2);
  g2.position.y = 170;
  g2.position.x = 870;
  g2.setYAxisTickSpacing(20);
  g2.setXAxisMax(5f);
  g2.setNoBorder();
  g2.setBackground(gb2);
  g2.setAxisColour(90, 90, 90);
  g2.setFontColour(90, 90, 90);
  g2.setXAxisTickSpacing(5.0);
  g2.setXAxisMinorTicks(5);
  g2.setXAxisLabel("");
  g2.setYAxisLabel("Degrees F");

  
  g3 = new Graph2D(this, 320, 80, false);
  g3.setYAxisMax(prt_rssi_max);
  g3.setYAxisMin(prt_rssi_min);
  g3.addTrace(r3);
  g3.position.y = 300;
  g3.position.x = 870;
  g3.setYAxisTickSpacing(20);
  g3.setXAxisMax(5f);
  g3.setNoBorder();
  g3.setBackground(gb3);
  g3.setAxisColour(90, 90, 90);
  g3.setFontColour(90, 90, 90);
  g3.setXAxisTickSpacing(5.0);
  g3.setXAxisMinorTicks(5);
  g3.setXAxisLabel("");
  g3.setYAxisLabel("RF Strength %");
  
  g4 = new Graph2D(this, 320, 80, false);
  g4.setYAxisMax(prt_cur_max);
  g4.setYAxisMin(prt_cur_min);
  g4.addTrace(r6);
  g4.position.y = 430;
  g4.position.x = 870;
  g4.setYAxisTickSpacing(50);
  g4.setXAxisMax(5f);
  g4.setNoBorder();
  g4.setBackground(gb4);
  g4.setAxisColour(90, 90, 90);
  g4.setFontColour(90, 90, 90);
  g4.setXAxisTickSpacing(5.0);
  g4.setXAxisMinorTicks(5);
  g4.setXAxisLabel("");
  g4.setYAxisLabel("Current mA");
  
  g5 = new Graph2D(this, 320, 80, false);
  g5.setYAxisMax(prt_vol_max);
  g5.setYAxisMin(prt_vol_min);
  g5.addTrace(r5);
  g5.addTrace(r4);
  g5.position.y = 560;
  g5.position.x = 870;
  g5.setYAxisTickSpacing(0.2);
  g5.setXAxisMax(5f);
  g5.setNoBorder();
  g5.setBackground(gb5);
  g5.setAxisColour(90, 90, 90);
  g5.setFontColour(90, 90, 90);
  g5.setXAxisTickSpacing(5.0);
  g5.setXAxisMinorTicks(5);
  g5.setXAxisLabel("");
  g5.setYAxisLabel("Voltage V");
}

/*=========================================
 Serial Functions
 =========================================*/
void autoConnect() {
  if (autoConnectSerial && !foundSerial) {
    int serialCount = Serial.list().length;
    if (serialCount > 0) {
      portChooser.captionLabel().set(Serial.list()[0]);
      serialPortNumber = 0;
      connectSwarm(0);
    }
  }
}
void resetConnection() {
  portChooser.captionLabel().set("Select Serial Port");
  serialPortNumber = -1;
  currentSerialDevice = "";
  if (foundSerial) {
    if (comPort != null) comPort.stop();
    logConsole("Lost Serial Connection to " + currentSerialDevice);
    foundSerial = false;
  }
}

void listSerialPorts() {
  logConsole("Finding SWARM Devices...");
  int serialCount = Serial.list().length;
  if (serialCount == 0) {
    portChooser.clear();
    logError("No Serial Devices Found");
    resetConnection();
    serialPortNumber = -1;
  } 
  else {
    logConsole("Found " + serialCount + " devices!");
    portChooser.clear();
    Serial.list();
    for (int i=0; i < serialCount; i++) {
      portChooser.addItem(Serial.list()[i], i);
      logConsole("Found " + Serial.list()[i]);
    }
    autoConnect();
  }
}

public void _listSerialTimer() {
  int serialCount = Serial.list().length;
  if (serialCount == 0) {
    portChooser.clear();
    resetConnection();
    portChooser.captionLabel().set("Select Serial Port");
    serialPortNumber = -1;
  } 
  else {
    portChooser.clear();
    boolean foundOurSerial = false;
    Serial.list();
    for (int i=0; i < serialCount; i++) {
      portChooser.addItem(Serial.list()[i], i);
      if ( Serial.list()[i].equals(currentSerialDevice) ) {
        foundOurSerial = true;
      }
    }
    if (!foundOurSerial && foundSerial) resetConnection();
    autoConnect();
  }

  //check for device connectivity
  Runnable task = new Runnable() {
    public void run() {
      _listSerialTimer();
    }
  };
  worker.schedule(task, 1000, TimeUnit.MILLISECONDS);
}

public void initializeSerial() {
  listSerialPorts();
  _listSerialTimer();
  _readSerial();
}

void connectSwarm(int serialDevice) {
  if (serialDevice < 0 || Serial.list().length <= serialDevice) {
    logError("Invalid device ID");
    resetConnection();
  } 
  else if (Serial.list().length == 0) {
    logError("No Serial Devices Found");
    resetConnection();
  } 
  else {
    logConsole("Connecting to " + Serial.list()[serialDevice]);
    try {
      if (comPort != null) comPort.stop();
      comPort = new Serial(this, Serial.list()[serialDevice], baudrate, 'N', 8, 1.0);
      comPort.buffer(bytesToRead * 3);
      logConsole("Connected!");
      currentSerialDevice = Serial.list()[serialDevice];
      serialPortNumber = serialDevice;
      foundSerial = true;
    } 
    catch (Exception e) {
      //leave blank
      logError("This device is busy!");
      resetConnection();
    }
    if (comPort == null) {
      logError("Error establishing connection.");
      resetConnection();
    }
  }
}

void useAutoConnect(float[] a) {
  autoConnectSerial = (a[0] == 1.0);
}

void controlEvent(ControlEvent theEvent) {
  if (theEvent.isGroup()) {
    portChooser.setOpen(true);
    String listName = theEvent.getName();
    if (listName.equals("portChooserD")) {
      serialPortNumber = (int)portChooser.getValue();
    }
    connectSwarm(serialPortNumber);
  }
}

void parseSerialFrame() {
  //time to dig into serialBuffer
  IR0 = ((int)serialBuffer[0] << 4) + ((int)(serialBuffer[1]&0xF0) >> 4);
  IR1 = ((int)(serialBuffer[1]&0x0F) << 8) + ((int)serialBuffer[2]);
  
  IR2 = ((int)serialBuffer[3] << 4) + ((int)(serialBuffer[4]&0xF0) >> 4);
  IR3 = ((int)(serialBuffer[4]&0x0F) << 8) + ((int)serialBuffer[5]);
  
  IR4 = ((int)serialBuffer[6] << 4) + ((int)(serialBuffer[7]&0xF0) >> 4);
  IR5 = ((int)(serialBuffer[7]&0x0F) << 8) + ((int)serialBuffer[8]);
  
  IR6 = ((int)serialBuffer[9] << 4) + ((int)(serialBuffer[10]&0xF0) >> 4);
  IR7 = ((int)(serialBuffer[10]&0x0F) << 8) + ((int)serialBuffer[11]);
  
  MEMS0 = ((int)serialBuffer[12] << 4) + ((int)(serialBuffer[13]&0xF0) >> 4);
  MEMS1 = ((int)(serialBuffer[13]&0x0F) << 8) + ((int)serialBuffer[14]);
  
  MEMS2 = ((int)serialBuffer[15] << 4) + ((int)(serialBuffer[16]&0xF0) >> 4);
  MEMS3 = ((int)(serialBuffer[16]&0x0F) << 8) + ((int)serialBuffer[17]);
  
  LDR = ((int)serialBuffer[18] << 4) + ((int)(serialBuffer[19]&0xF0) >> 4);
  
  ES3 = ((int)(serialBuffer[19]&0x08) >> 3) == 1;
  ES2 = ((int)(serialBuffer[19]&0x04) >> 2) == 1;
  ES1 = ((int)(serialBuffer[19]&0x02) >> 1) == 1;
  ES0 = ((int)(serialBuffer[19]&0x01)) == 1;
  
  IMU_X = ((int)serialBuffer[20] << 8) + (int)serialBuffer[21];
  IMU_Y = ((int)serialBuffer[22] << 8) + (int)serialBuffer[23];
  IMU_Z = ((int)serialBuffer[24] << 8) + (int)serialBuffer[25];
  
  TEMP = ((int)serialBuffer[26] << 8) + (int)serialBuffer[27];
  
  CURR = ((int)serialBuffer[28] << 4) + ((int)(serialBuffer[29]&0xF0) >> 4);
  RSSI = ((int)(serialBuffer[29]&0x0F) << 8) + ((int)serialBuffer[30]);
  
  VOL0 = ((int)serialBuffer[31] << 4) + ((int)(serialBuffer[32]&0xF0) >> 4);
  VOL1 = ((int)(serialBuffer[32]&0x0F) << 8) + ((int)serialBuffer[33]);
  
  ES3 = ((int)(serialBuffer[34]&0x80) >> 7) == 1;
  ES2 = ((int)(serialBuffer[34]&0x40) >> 6) == 1;
  ES1 = ((int)(serialBuffer[34]&0x20) >> 5) == 1;
  ES0 = ((int)(serialBuffer[34]&0x10) >> 4) == 1;
  
  DP0 = ((int)(serialBuffer[34]&0x08) >> 3) == 1;
  DP1 = ((int)(serialBuffer[34]&0x04) >> 2) == 1;
  DP2 = ((int)(serialBuffer[34]&0x02) >> 1) == 1;
  DP3 = ((int)(serialBuffer[34]&0x01)) == 1;
  
}

void readSerial() {
  if (comPort.available() != 0) {
    previousByte = (byte)lastByte;
    lastByte = (byte)inByte;
    inByte = (byte)comPort.read();
    serialBtyesRec++;
    if (previousByte == 0x3D && lastByte == 0x3D && inByte == 0x3D ) {
      //begin parsing serial...
      if (inPacket && placeInFrame > 3 && placeInFrame < bytesToRead) {
        logError("Dropped Malformed Packet " + (packetNumber-1));
        droppedPackets++;
      }
      placeInFrame = 0;
      logConsole("Starting Packet " + packetNumber++);
      inPacket = true;
    }
    if (placeInFrame < bytesToRead) serialBuffer[placeInFrame] = inByte;
    placeInFrame++;
    if (placeInFrame >= bytesToRead) {
      if (inPacket) {
        logConsole("Recieved Packet " + (packetNumber-1));
        parseSerialFrame();
        placeInFrame = 0;
        inPacket = false;
      }
    }
  }
}

void _readSerial() {
  readSerial();
  //check for device connectivity
  Runnable task = new Runnable() {
    public void run() {
      _readSerial();
    }
  };
  worker.schedule(task, 1, TimeUnit.MILLISECONDS);
}

void _sendSerial(String toSend) {
  if (foundSerial) {
    comPort.write(toSend);
    comPort.write(0x3D);
    logConsole("Sending => " + toSend);
    cp5.get(Textfield.class,"serialToSend").clear();
  } else logError("No Device Connected");
}

/*=========================================
 Initialize Graphics
 =========================================*/
public void setupGraphics() {
  f = createFont("Arial", 16, true); // Arial, 16 point, anti-aliasing on

  consoleTextarea = cp5.addTextarea("consoleLog").setPosition(10, 710).setSize(585, 150).setFont(createFont("arial", 12)).setLineHeight(14).setColor(color(200)).setColorBackground(color(20)).setColorForeground(color(20)).showScrollbar().setScrollForeground(color(70)).setScrollBackground(color(10));

  portChooser = cp5.addDropdownList("portChooserD").setPosition(615, 763).setWidth(260);
  portChooser.setBackgroundColor(color(190));
  portChooser.setItemHeight(20);
  portChooser.setBarHeight(20);
  portChooser.captionLabel().set("Select Serial Port");
  portChooser.captionLabel().style().marginTop = 5;
  portChooser.captionLabel().style().marginLeft = 3;
  portChooser.valueLabel().style().marginTop = 3;
  portChooser.setValue(9999.0);
  portChooser.setColorActive(color(255, 128));
  portChooser.setOpen(true).setMoveable(false);

  cp5.addButton("refreshUSB").setPosition(885, 800).setSize(80, 20).setCaptionLabel("Refresh Devices").captionLabel().style().marginLeft = 2;
  cp5.addButton("connectUSB").setPosition(885, 830).setSize(80, 20).setCaptionLabel("Reset").captionLabel().style().marginLeft = 5;
  cp5.addButton("disconnectUSB").setPosition(885, 860).setSize(80, 20).setCaptionLabel("Disconnect").captionLabel().style().marginLeft = 4;
  cp5.addToggle("autoConnectSerial").setPosition(885, 742).setSize(80, 20).setCaptionLabel("Auto Connect");
  
  sendDialog = cp5.addTextfield("serialToSend");
  sendDialog.setPosition(20,860)
     .setSize(475,20)
     .setFont(createFont("arial",16))
     .setAutoClear(true)
     .getCaptionLabel().setVisible(false)
     ;
       
  cp5.addBang("sendSerial")
     .setPosition(505,860)
     .setSize(80,20).setCaptionLabel("Send")
     .getCaptionLabel().align(ControlP5.CENTER, ControlP5.CENTER)
     ;
     
  cp5.getTooltip().setDelay(500);
  cp5.getTooltip().register("portChooserD", "Click on a device to connect.");
  cp5.getTooltip().register("connectUSB", "Reset connection to selected Device.");
  cp5.getTooltip().register("refreshUSB", "Refresh device list manually.");
  cp5.getTooltip().register("disconnectUSB", "Disconnect from device.");
  cp5.getTooltip().register("serialToSend", "Send a serial command to device.");
  cp5.getTooltip().register("sendSerial", "Send a serial command to device.");
  cp5.getTooltip().register("autoConnectSerial", "Enable/Disable automatic device connections.");
}

/*=========================================
 UI Functions
 =========================================*/
 public void sendSerial() {
    _sendSerial(sendDialog.getText());
}
public void serialToSend(String theText) {
  _sendSerial(theText);
}

void disconnectUSB() {
  logConsole("Disconnecting Serial Device..");
  resetConnection();
}
void connectUSB() {
  logConsole("Resetting connection to Serial Device..");
  resetConnection();
  autoConnect();
}
void refreshUSB() {
  listSerialPorts();
}

/*=========================================
 Main Initialization
 =========================================*/
public void setup() {
  initializeData();
  
  frame.setTitle("SWARM Robotics Debugger");
  PFont font = createFont("arial", 12);
  size(1200, 900);
  noStroke();
  
  cp5 = new ControlP5(this);
  cp5.setAutoDraw(false);
  setupGraphics();
  logConsole("Graphics Initialized");

  logConsole("Initializing Serial Communications");
  
  initializeSerial();
  
  setupGraphs();

  logConsole("Finished Initialization");
}

/*=========================================
 Global Output Renderer
 =========================================*/
public void draw() {
  background(0);
  
  fill(20);
  rect(10, 860, 585, 30);
  fill(40);
  rect(605, 710, 585, 180);
  fill(10);
  rect(615, 763, 260, 117);

  textFont(f);
  fill(255);
  text("Active Device List:", 615, 733);
  text("Tools:", 885, 733);

  text("Statistics:", 975, 733);
  text("Recieved: " + (serialBtyesRec / 1000.0) + " KB", 975, 795);
  text("Packets: " + ((packetNumber-1) / 1000.0) + " kP", 975, 815);
  text("Dropped: " + (droppedPackets / 1000.0) + " kP", 975, 835);
  if (packetNumber == 1) text("PDR: 100%", 975, 875);
  else text("PDR: " + (((packetNumber-droppedPackets-1)*1.0)/((packetNumber-1)*1.0)*100) + "%", 975, 875);
  if (inPacket) {
    text("Read State: Recv", 975, 855);
  } else {
    text("Read State: Idle", 975, 855);
  }
  text(currentSerialDevice + " @ " + (baudrate / 1000.0) + " KBaud", 975, 755);

  text("LDR Brightness", 870, 36);
  text("Temperature (F)", 870, 166);
  text("RF Signal Strength", 870, 296);
  text("Stepper Current", 870, 426);
  text("Battery Voltage", 870, 556);

  text("Time (s)", 1000, 675);
  cp5.draw();
  
  g1.draw();
  g2.draw();
  g3.draw();
  g4.draw();
  g5.draw();
}

