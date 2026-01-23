flowchart TD
  A([autoCorrect(targetX, targetY, targetHead, timeoutMs, maxSpeed)]) --> EN{autoCorrectEnabled_?}
  EN -- No --> END([Return])
  EN -- Yes --> I[iteration=0<br/>maxIterations=2] --> L{{Loop}}

  L --> E[Compute error<br/>dx=targetX-pose.x<br/>dy=targetY-pose.y<br/>dist=hypot(dx,dy)<br/>dHead=angleDiff(targetHead, headingDeg())]

  E --> EX{Exit?<br/>dist<EXIT_DIST and |dHead|<EXIT_ANG}
  EX -- Yes --> END
  EX -- No --> LIM{iteration >= maxIterations?}
  LIM -- Yes --> END
  LIM -- No --> H{Need heading fix?<br/>|dHead|>ENTER_ANG}

  H -- Yes --> H1[turnTo(targetHead, timeout/3)] --> R1
  H -- No --> R1[Recalc dx,dy,dist]

  R1 --> B[Check behind<br/>localY = dx*sin(head)+dy*cos(head)]
  B --> BK{localY < -0.02?}
  BK -- Yes --> BK1[drive(-BACKUP_M, timeout/4, maxSpeed)] --> LAT
  BK -- No --> LAT

  LAT[localX = dx*cos(head)-dy*sin(head)] --> LCHK{|localX| > ENTER_POS?}
  LCHK -- Yes --> L1[turnBy( localX>0 ? 90 : -90, timeout/4)]
  L1 --> L2[drive( min(|localX|,MAX_STEP_M), timeout/4, maxSpeed)]
  L2 --> L3[turnTo(targetHead, timeout/4)] --> FWD
  LCHK -- No --> FWD

  FWD[Recalc; localY = dx*sin(head)+dy*cos(head)] --> FCHK{|localY| > ENTER_POS?}
  FCHK -- Yes --> F1[drive( clamp(localY,-MAX_STEP,MAX_STEP), timeout/4, maxSpeed)] --> FIN
  FCHK -- No --> FIN

  FIN[dHead=angleDiff(targetHead, headingDeg())] --> FANG{|dHead| > EXIT_ANG?}
  FANG -- Yes --> F2[turnTo(targetHead, timeout/4)] --> INC
  FANG -- No --> INC[iteration++]
  INC --> L
