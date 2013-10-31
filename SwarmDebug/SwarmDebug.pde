import controlP5.*;
import org.gwoptics.graphics.graph2D.Graph2D;
import org.gwoptics.graphics.graph2D.traces.ILine2DEquation;
import org.gwoptics.graphics.graph2D.traces.RollingLine2DTrace;
import processing.serial.*;
import java.util.Calendar;
import java.text.SimpleDateFormat;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;


Serial myPort;
ControlP5 cp5;
RollingLine2DTrace r,r2,r3;
Graph2D g;



public void setup() {
  size(1200,800, P2D);
  noStroke();
  cp5 = new ControlP5(this);
  cp5.setAutoDraw(false);
}

public void draw() {
  background(0);
  
  
  cp5.draw();
  //g.draw();
}
