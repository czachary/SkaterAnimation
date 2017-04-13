
class pts // class for manipulaitng and displaying pointclouds or polyloops in 3D 
  { 
    int maxnv = 16000;                 //  max number of vertices
    pt[] G = new pt [maxnv];           // geometry table (vertices)
    char[] L = new char [maxnv];             // labels of points
    vec [] LL = new vec[ maxnv];  // displacement vectors
    Boolean loop=true;          // used to indicate closed loop 3D control polygons
    int pv =0,     // picked vertex index,
        iv=0,      //  insertion vertex index
        dv = 0,   // dancer support foot index
        nv = 0,    // number of vertices currently used in P
        pp=1; // index of picked vertex

  pts() {}
  pts declare() 
    {
    for (int i=0; i<maxnv; i++) G[i]=P(); 
    for (int i=0; i<maxnv; i++) LL[i]=V(); 
    return this;
    }     // init all point objects
  pts empty() {nv=0; pv=0; return this;}                                 // resets P so that we can start adding points
  pts addPt(pt P, char c) { G[nv].setTo(P); pv=nv; L[nv]=c; nv++;  return this;}          // appends a new point at the end
  pts addPt(pt P) { G[nv].setTo(P); pv=nv; L[nv]='f'; nv++;  return this;}          // appends a new point at the end
  pts addPt(float x,float y) { G[nv].x=x; G[nv].y=y; pv=nv; nv++; return this;} // same byt from coordinates
  pts copyFrom(pts Q) {empty(); nv=Q.nv; for (int v=0; v<nv; v++) G[v]=P(Q.G[v]); return this;} // set THIS as a clone of Q

  pts resetOnCircle(int k, float r)  // sets THIS to a polyloop with k points on a circle of radius r around origin
    {
    empty(); // resert P
    pt C = P(); // center of circle
    for (int i=0; i<k; i++) addPt(R(P(C,V(0,-r,0)),2.*PI*i/k,C)); // points on z=0 plane
    pv=0; // picked vertex ID is set to 0
    return this;
    } 
  // ********* PICK AND PROJECTIONS *******  
  int SETppToIDofVertexWithClosestScreenProjectionTo(pt M)  // sets pp to the index of the vertex that projects closest to the mouse 
    {
    pp=0; 
    for (int i=1; i<nv; i++) if (d(M,ToScreen(G[i]))<=d(M,ToScreen(G[pp]))) pp=i; 
    return pp;
    }
  pts showPicked() {show(G[pv],23); return this;}
  pt closestProjectionOf(pt M)    // Returns 3D point that is the closest to the projection but also CHANGES iv !!!!
    {
    pt C = P(G[0]); float d=d(M,C);       
    for (int i=1; i<nv; i++) if (d(M,G[i])<=d) {iv=i; C=P(G[i]); d=d(M,C); }  
    for (int i=nv-1, j=0; j<nv; i=j++) { 
       pt A = G[i], B = G[j];
       if(projectsBetween(M,A,B) && disToLine(M,A,B)<d) {d=disToLine(M,A,B); iv=i; C=projectionOnLine(M,A,B);}
       } 
    return C;    
    }

  // ********* MOVE, INSERT, DELETE *******  
  pts insertPt(pt P) { // inserts new vertex after vertex with ID iv
    for(int v=nv-1; v>iv; v--) {G[v+1].setTo(G[v]);  L[v+1]=L[v];}
     iv++; 
     G[iv].setTo(P);
     L[iv]='f';
     nv++; // increments vertex count
     return this;
     }
  pts insertClosestProjection(pt M) {  
    pt P = closestProjectionOf(M); // also sets iv
    insertPt(P);
    return this;
    }
  pts deletePicked() 
    {
    for(int i=pv; i<nv; i++) 
      {
      G[i].setTo(G[i+1]); 
      L[i]=L[i+1]; 
      }
    pv=max(0,pv-1); 
    nv--;  
    return this;
    }
  pts setPt(pt P, int i) { G[i].setTo(P); return this;}
  
  pts drawBalls(float r) {for (int v=0; v<nv; v++) show(G[v],r); return this;}
  pts showPicked(float r) {show(G[pv],r); return this;}
  pts drawClosedCurve(float r) 
    {
    fill(dgreen);
    for (int v=0; v<nv; v++) show(G[v],r*3);    
    fill(magenta);
    for (int v=0; v<nv-1; v++) stub(G[v],V(G[v],G[v+1]),r,r);  
    stub(G[nv-1],V(G[nv-1],G[0]),r,r);
    pushMatrix(); //translate(0,0,1); 
    scale(1,1,0.03);  
    fill(grey);
    for (int v=0; v<nv; v++) show(G[v],r*3);    
    for (int v=0; v<nv-1; v++) stub(G[v],V(G[v],G[v+1]),r,r);  
    stub(G[nv-1],V(G[nv-1],G[0]),r,r);
    popMatrix();
    return this;
    }
  pts set_pv_to_pp() {pv=pp; return this;}
  pts movePicked(vec V) { G[pv].add(V); return this;}      // moves selected point (index p) by amount mouse moved recently
  pts setPickedTo(pt Q) { G[pv].setTo(Q); return this;}      // moves selected point (index p) by amount mouse moved recently
  pts moveAll(vec V) {for (int i=0; i<nv; i++) G[i].add(V); return this;};   
  pt Picked() {return G[pv];} 
  pt Pt(int i) {if(0<=i && i<nv) return G[i]; else return G[0];} 

  // ********* I/O FILE *******  
 void savePts(String fn) 
    {
    String [] inppts = new String [nv+1];
    int s=0;
    inppts[s++]=str(nv);
    for (int i=0; i<nv; i++) {inppts[s++]=str(G[i].x)+","+str(G[i].y)+","+str(G[i].z)+","+L[i];}
    saveStrings(fn,inppts);
    };
  
  void loadPts(String fn) 
    {
    println("loading: "+fn); 
    String [] ss = loadStrings(fn);
    String subpts;
    int s=0;   int comma, comma1, comma2;   float x, y;   int a, b, c;
    nv = int(ss[s++]); print("nv="+nv);
    for(int k=0; k<nv; k++) 
      {
      int i=k+s; 
      //float [] xy = float(split(ss[i],",")); 
      String [] SS = split(ss[i],","); 
      G[k].setTo(float(SS[0]),float(SS[1]),float(SS[2]));
      L[k]=SS[3].charAt(0);
      }
    pv=0;
    };
 
  // Dancer
  void setPicekdLabel(char c) {L[pp]=c;}
  


  void setFifo() 
    {
    _LookAtPt.reset(G[dv],60);
    }              


  void next() {dv=n(dv);}
  int n(int v) {return (v+1)%nv;}
  int p(int v) {if(v==0) return nv-1; else return v-1;}
  
  
  /////**MODULE 1**********
  
  //Quadratic
  pts subdivideQuadraticInto(pts Q) 
    {
    Q.empty();
    for(int i=0; i<nv; i++)
      {
      Q.addPt(P(G[i], 0.25, G[n(i)])); 
      Q.addPt(P(G[i], 0.75, G[n(i)])); 
      }
    return Q;
    }  
 
   //CUBIC
   pts subdivideCubicInto(pts Q) 
    {
    Q.empty();
    for(int i=0; i<nv; i++)
      {
        pt d1 =P(G[i], 0.25, G[p(i)]);
        pt d2 =P(G[i], 0.25, G[n(i)]);
        Q.addPt(P(d1, 0.5, d2));
        Q.addPt(P(G[i], 0.5, G[n(i)]));
      }
    return this;
    }
    
    //FOURPOINT
    pts subdivideFourPointInto(pts Q) 
    {
    Q.empty();
    for(int i=0; i<nv; i++)
      {
        Q.addPt(G[i]);
        pt S = P(G[p(i)]).mul(-1.0/16);
        S.add(P(G[i]).mul(9.0/16));
        S.add(P(G[n(i)]).mul(9.0/16));
        S.add(P(G[n(n(i))]).mul(-1.0/16));
        Q.addPt(P(S));
      }
    return this;
    }
    
    //QUINTIC
    pts subdivideQuinticInto(pts Q) 
    {
    Q.empty();
    pt dual1a, dual1b, dual1c, dual1d;
    pt dual2a, dual2b, dual2c, dual2d;
    pt dual3a, dual3b, dual3c;
    for(int i=0; i<nv; i++)
      {
        //refine + dual1 in this code block
        dual1a = P(G[i], 0.25, G[p(i)]);
        dual1b = P(G[i], 0.25, G[n(i)]);
        dual1c = P(G[i], 0.75, G[n(i)]);
        dual1d = P(G[i], 0.25, G[n(n(i))]);
        
        //dual2
        dual2a = P(G[i], 0.5, G[p(i)]);
        dual2b = P(dual1a, 0.5, dual1b);
        dual2c = P(G[i], 0.5, G[n(i)]);
        dual2d = P(dual1c, 0.5, dual1d);
        
        //dual3
        dual3a = P(dual2a, 0.5, dual2b);
        dual3b = P(dual2b, 0.5, dual2c);
        dual3c = P(dual2c, 0.5, dual2d);
        
        //dual4
        Q.addPt(P(dual3a, 0.5, dual3b));
        Q.addPt(P(dual3b, 0.5, dual3c));
      }
    return this;
    }
    
    //JAREK
    pts subdivideJarekInto(pts Q) 
    {
    Q.empty();
    pt d1a, d1b, d1c, d1d;
    pt d2a, d2b, d2c, d2d;
    for(int i=0; i<nv; i++)
      {
        //refine, dual
        d1a =P(G[i], 0.25, G[p(i)]);
        d1b =P(G[i], 0.25, G[n(i)]);
        d1c =P(G[i], 0.75, G[n(i)]);
        d1d =P(G[i], 0.25, G[n(n(i))]);
        
        //dual2
        d2a = P(G[i], 0.5, G[p(i)]);
        d2b = P(d1a, 0.5, d1b);
        d2c = P(G[i], 0.5, G[n(i)]);
        d2d = P(d1c, 0.5, d1d);
        
        //tuck(-1)
        vec L = V(d2b, d2a).add(V(d2b, d2c)).mul(0.25);
        Q.addPt(P(d2b, -1.0, L));
        
        L = V(d2c, d2b).add(V(d2c, d2d)).mul(0.25);
        Q.addPt(P(d2c, -1.0, L));
        
        
      }
    return this;
    }

  
  
  //MODULE 2: calculate F(t) and display arrow (or call to display skater)
  void displayCurve() 
      {
      if(showCurve) {fill(yellow); for (int j=0; j<nv; j++) caplet(G[j],6,G[n(j)],6); } // display subdivided curve
      pt[] B = new pt [nv];

      
      for (int j=0; j<nv; j++) //Calculate F(t)
      {
        vec accel = V(G[j],G[n(j)]).add(V(G[j],G[p(j)]));
        B[j] = P(G[j]).add(accel.mul(-1*level*accelMultiplier)).add(V(0,0,-1).mul(gravConstant));
      }

      if(showPath) {fill(cyan); for (int j=0; j<nv; j++) caplet(B[j],6,B[n(j)],6);} //display F(t)
      if(showKeys) {fill(green); for (int j=0; j<nv; j+=4) arrow(B[j],G[j],3);}
      
      
      if(animating)
      {
        f++;
        if (f>fperstep)
        {
          f=0;
          currstep = n(n(currstep));
          stepNum++;
        }
      }
      t=(f*1.0)/fperstep;
      
      //EXTRACREDIT: RUNNER
      if(showSkater) 
        {
          if(stepNum%2 == 1) leftIsSupport = true;
          else if (stepNum%2 == 0) leftIsSupport = false;
          
          pt rightFoot = P();
          pt leftFoot = P();
          pt basePoint = P();
          pt origCurvePoint = P();
          
          if (t<1.0/3) //COLLECT
          {
            if(leftIsSupport)
            {
              leftFoot = P(B[n(currstep)]);
              if (t<1.0/6) rightFoot = P(B[p(currstep)], t/(1.0/6), B[currstep]);
              else rightFoot = P(B[currstep], (t-(1.0/6))/(1.0/6), B[n(currstep)]);
            }
            else if(!leftIsSupport)
            {
              rightFoot = P(B[n(currstep)]);
              if (t<1.0/6) leftFoot = P(B[p(currstep)], t/(1.0/6), B[currstep]);
              else leftFoot = P(B[currstep], (t-(1.0/6))/(1.0/6), B[n(currstep)]);
            }
            
            pt startCollect = P(B[currstep], 1.0/3, B[n(currstep)]);
            basePoint = P(startCollect, t/(1.0/3), B[n(currstep)]);
            
            pt startCollectOrigCurve = P(G[currstep], 1.0/3, G[n(currstep)]);
            origCurvePoint = P(startCollectOrigCurve, t/(1.0/3), G[n(currstep)]);
          }
          else if (t>=1.0/3 && t<2.0/3) //AIM
          {
            if(leftIsSupport)
            {
              leftFoot = P(B[n(currstep)]);
              rightFoot = P(B[n(currstep)], (t-(1.0/3))/(1.0/3), B[n(n(n(currstep)))]);
            }
            else if (!leftIsSupport)
            {
              rightFoot = P(B[n(currstep)]);
              leftFoot = P(B[n(currstep)], (t-(1.0/3))/(1.0/3), B[n(n(n(currstep)))]);
            }
            endAim = P(B[n(currstep)], 2.0/3, B[n(n(currstep))]);
            basePoint = P(B[n(currstep)], (t-(1.0/3))/(1.0/3), endAim);
            
            endAimOrigCurve = P(G[n(currstep)], 2.0/3, G[n(n(currstep))]);
            origCurvePoint = P(G[n(currstep)], (t-(1.0/3))/(1.0/3), endAimOrigCurve);
            
          } else if (t>=2.0/3) //TRANSFER
          {
            if(leftIsSupport) { rightFoot = P(B[n(n(n(currstep)))]); leftFoot = P(B[n(currstep)]); }
            else if (!leftIsSupport) { leftFoot = P(B[n(n(n(currstep)))]); rightFoot = P(B[n(currstep)]); }
            
            endTransfer = P(B[n(n(currstep))], 1.0/3, B[n(n(n(currstep)))]);
            basePoint = P(endAim, (t-(2.0/3))/(1.0/3), endTransfer);
            
            pt endTransferOrigCurve = P(G[n(n(currstep))], 1.0/3, G[n(n(n(currstep)))]);
            origCurvePoint = P(endAimOrigCurve, (t-(2.0/3))/(1.0/3), endTransferOrigCurve);

          }
          
          
          pt BodyCenter = P(basePoint, U(basePoint,origCurvePoint).mul(skaterHeight));
          displayUpperBody(BodyCenter, U(basePoint,origCurvePoint), V(B[currstep], B[n(currstep)]));
          
          fill(green); displaySupportLeg(leftFoot, leftHip, U(basePoint,origCurvePoint), V(B[currstep], B[n(currstep)]));
          fill(red); displaySupportLeg(rightFoot, rightHip, U(basePoint,origCurvePoint), V(B[currstep], B[n(currstep)]));

        }
      else
        {
          fill(red);
          pt basePoint = P(B[currstep], t, B[n(currstep)]);
          pt origCurvePoint = P(G[currstep], t, G[n(currstep)]);
          arrow(basePoint,U(basePoint,origCurvePoint).mul(skaterHeight),20);
        } 
      
    }
    
    
    void displaySupportLeg(pt S, pt hip, vec bodyDir, vec dancerFacing) //compute leg points with heel on ground (loses hip angle)
      {
        //BALL/TOE
        sphere(S,footRadius);
        pt toe =   P(S,5,U(dancerFacing));
        caplet(S,footRadius,toe,1);
        
        //HEEL
        pt heel = P(S,_eb,U(dancerFacing).mul(-1));
        heel = P(heel, heelRadius,bodyDir);
        sphere(heel, heelRadius);
        caplet(heel, heelRadius,S,footRadius);
        
        //////ANKLE
        float a4 = acos(((_ab*_ab)+(_ae*_ae)-(_eb*_eb))/(2*_ab*_ae));
        float b = asin((_ae*sin(a4))/_eb);
        pt ankle = P(S.x,S.y,S.z);
        ankle = ankle.add(_ab, R(U(S,heel), b, U(S,heel),bodyDir));
        caplet(S,footRadius,ankle,ankleRadius);
        caplet(heel, heelRadius,ankle,ankleRadius);
        sphere(ankle,ankleRadius);
        
        // KNEE
        pt K = triangleTip(ankle,_ka,hip,_hk,true, dancerFacing);
        sphere(K,kneeRadius);
        caplet(hip,hipRadius,K,kneeRadius);
        caplet(K,kneeRadius,ankle,ankleRadius);
    
      }
    
    
    void displayUpperBody(pt center, vec bodyDir, vec dancerFacing)
      {
        //BODY/PELVIS
        pt Pelvis = P(center,pelvisHeight,bodyDir, pelvisForward,U(dancerFacing)); 
        fill(blue); sphere(Pelvis,pelvisRadius);
        
        //HIPS
        vec right = N(bodyDir,U(dancerFacing));
        rightHip =  P(center,hipSpread,right); sphere(rightHip,hipRadius);
        leftHip =  P(center,-hipSpread,right); sphere(leftHip,hipRadius);
        caplet(Pelvis,pelvisRadius,leftHip,hipRadius);  
        caplet(Pelvis,pelvisRadius,rightHip,hipRadius);
        
        //SHOULDERS
        pt LeftShoulder = P(center, shoulderHeight, bodyDir, -midToShoulderDist, right);
        pt RightShoulder = P(center, shoulderHeight, bodyDir, midToShoulderDist, right);
        sphere(LeftShoulder, shoulderRadius);
        sphere(RightShoulder, shoulderRadius);
        caplet(Pelvis, waistRadius, LeftShoulder, shoulderRadius);
        caplet(Pelvis, waistRadius, RightShoulder, shoulderRadius);
        caplet(LeftShoulder, shoulderRadius, RightShoulder, shoulderRadius);
        
        //CHEST
        pt LeftChest = P(Pelvis, chestHeight, bodyDir, -chestDist, right);
        pt RightChest = P(Pelvis, chestHeight, bodyDir, chestDist, right);
        sphere(LeftChest, chestRadius);
        sphere(RightChest, chestRadius);
        caplet(LeftChest, chestRadius, LeftShoulder, shoulderRadius);
        caplet(RightChest, chestRadius, RightShoulder, shoulderRadius);
        caplet(LeftChest, chestRadius, P(center, 4, U(dancerFacing)), waistRadius);
        caplet(RightChest, chestRadius, P(center, 4, U(dancerFacing)), waistRadius);
        
        //NECK
        pt Neck = P(center, shoulderHeight, bodyDir, neckHeight, bodyDir);
        sphere(Neck, neckRadius);
        caplet(Neck, neckRadius, P(center, shoulderHeight, bodyDir), neckRadius);
        
        //HEAD
        pt Head = P(Neck, headHeight, bodyDir);
        pt Chin = P(Neck, chinDistUp, bodyDir, chinDistForward, U(dancerFacing));
        sphere(Head, headRadius);
        sphere(Chin, chinRadius);
        caplet(Head, headRadius, Chin, chinRadius);
        
      }

    pt triangleTip(pt A, float rA, pt B, float rB, boolean flip, vec dancerFacing)
      {
        pt tip = P(0,0);
        
        float d = d(A,B);
        float a = ((rB*rB)-(rA*rA)+(d*d))/(2*d);
        float h = sqrt((rB*rB)-(a*a));
        tip = P(B, (a/d), V(B,A));
        
        vec dirTri = V(A,tip);
        vec orthogVec = N(N(dirTri,dancerFacing),dirTri);
        
        tip = P(tip,h,U(orthogVec).mul(-1));
        if (flip) { tip = P(tip,2*h,U(orthogVec)); }
        
        return tip;
      }
        

} // end of pts class