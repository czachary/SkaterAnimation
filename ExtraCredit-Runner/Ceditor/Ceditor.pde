//  ******************* Project 4 - Christina Zachary Fall 2016 ***********************
Boolean 
  animating=false, 
  PickedFocus=false, 
  center=true, 
  track=false, 
  showViewer=false, 
  showBalls=false, 
  showControl=false, 
  showCurve=false, 
  showPath=true, 
  showKeys=true, 
  showSkater=true, 
  scene1=false,
  solidBalls=false,
  showCorrectedKeys=true,
  showQuads=true,
  showVecs=true,
  showTube=false;
float 
  t=0, 
  s=0;
int
  f=0, maxf=2*30, level=4, method=3, currstep=0, fperstep=7;
String SDA = "angle";
float defectAngle=0;
pts P = new pts(); // polyloop in 3D
pts Q = new pts(); // second polyloop in 3D
pts R = new pts(); // inbetweening polyloop L(P,t,Q);


int gravConstant = 50;
int accelMultiplier = 3;
int skaterHeight = 100;

pt leftHip = P(), rightHip = P();
boolean leftIsSupport = true;
pt endAim = P(), endTransfer = P(), endAimOrigCurve = P();
int stepNum = 1;

int saveLeftPos = -1, saveRightPos = -1;

//UPPER BODY Control Variables
float bodyHeight = 100, hipRadius=12, waistRadius = 10; // height of body center B
float hipSpread = hipRadius; // half-displacement between hips
float shoulderHeight = 50, midToShoulderDist = 20, shoulderRadius = 8;
float chestHeight = 30, chestDist = 10, chestRadius = 11, neckHeight = 15, neckRadius = 6;
float headHeight = 8, headRadius = 11, chinDistForward = 6, chinRadius = 6, chinDistUp = 2;
float pelvisHeight=10, pelvisForward=hipRadius/(2.5), pelvisRadius=hipRadius*1.1; // vertical distance form BodyCenter to Pelvis

//LEG Control Variables
float footRadius=3, kneeRadius = 6, heelRadius = 3 ; // radius of foot, knee, hip
float ankleBackward=10, ankleInward=4, ankleUp=6, ankleRadius=4; // ankle position with respect to footFront and size
float LeftKneeForward = 20; // arbitrary knee offset for mid (B,H)
float _hipAngle=-PI/12, _hk = 60, _ab = 15, _ka=50, _ae=4, _eb=16;


  
void setup() {
  myFace = loadImage("data/pic.JPG");  // load image from file pic.jpg in folder data *** replace that file with your pic of your own face
  textureMode(NORMAL);          
  size(900, 900, P3D); // P3D means that we will do 3D graphics
  P.declare(); Q.declare(); R.declare();// P is a polyloop in 3D: declared in pts
  //P.resetOnCircle(6,100); Q.copyFrom(P); // use this to get started if no model exists on file: move points, save to file, comment this line
  P.loadPts("data/pts"); // Q.loadPts("data/pts2"); // loads saved models from file (comment out if they do not exist yet)
  noSmooth();
  frameRate(30);
  }

void draw() {
  background(255);
  hint(ENABLE_DEPTH_TEST); 
  pushMatrix();   // to ensure that we can restore the standard view before writing on the canvas
  setView();  // see pick tab
  showFloor(); // draws dance floor as yellow mat
  doPick(); // sets Of and axes for 3D GUI (see pick Tab)
  P.SETppToIDofVertexWithClosestScreenProjectionTo(Mouse()); // for picking (does not set P.pv)
 
  R.copyFrom(P); 
  for(int i=0; i<level; i++) 
    {
    Q.copyFrom(R);
    if(method==4) {Q.subdivideQuinticInto(R);}
    if(method==3) {Q.subdivideCubicInto(R); }
    if(method==2) {Q.subdivideJarekInto(R); }
    if(method==1) {Q.subdivideFourPointInto(R);}
    if(method==0) {Q.subdivideQuadraticInto(R); }
    }
  R.displayCurve();
  
  fill(blue); if(showCurve) Q.drawClosedCurve(3);
  if(showControl) {fill(grey); P.drawClosedCurve(3);}  // draw control polygon - magenta
  //fill(yellow,100); P.showPicked(); 
  


  //if(track) F=_LookAtPt.move(X(t)); // lookAt point tracks point X(t) filtering for smooth camera motion (press'.' to activate)
 
  popMatrix(); // done with 3D drawing. Restore front view for writing text on canvas
  hint(DISABLE_DEPTH_TEST); // no z-buffer test to ensure that help text is visible
    if(method==4) scribeHeader("Quintic UBS",2);
    if(method==3) scribeHeader("Cubic UBS",2);
    if(method==2) scribeHeader("Jarek J-spline",2);
    if(method==1) scribeHeader("Four Points",2);
    if(method==0) scribeHeader("Quadratic UBS",2);

  // used for demos to show red circle when mouse/key is pressed and what key (disk may be hidden by the 3D model)
  if(keyPressed) {stroke(red); fill(white); ellipse(mouseX,mouseY,26,26); fill(red); text(key,mouseX-5,mouseY+4);}
  if(scribeText) {fill(black); displayHeader();} // dispalys header on canvas, including my face
  if(scribeText && !filming) displayFooter(); // shows menu at bottom, only if not filming
  if(filming && (animating || change)) saveFrame("FRAMES/F"+nf(frameCounter++,4)+".tif");  // save next frame to make a movie
  change=false; // to avoid capturing frames when nothing happens (change is set uppn action)
  }