```mermaid
flowchart TD
  A([drive(distM, timeoutMs, maxSpeedPct)]) --> B[Init<br/>holdHead = headingDeg()<br/>startDist = avgEncDist()<br/>targetDist = startDist + distM<br/>distPID: SP=targetDist, reset(startDist)<br/>headPID: SP=0, reset(0)<br/>vCmd=0, wCmd=0<br/>stopLatch=false, crossed=false<br/>elapsed=0, settled=0, prevErr=targetDist-startDist]

  B --> L{{Loop}}

  L --> T{elapsed >= timeoutMs?}
  T -- Yes --> X[Cleanup<br/>stopDrive(brake)] --> END([Return])

  T -- No --> S[Sense<br/>currDist=avgEncDist()<br/>currHead=headingDeg()<br/>distErr=targetDist-currDist<br/>headErr=angleDiff(holdHead,currHead)]

  S --> H{stopLatch?}

  %% ---------------- STOP & HOLD ----------------
  H -- Yes --> SH1[STOP-HOLD: heading only<br/>if |distErr| > stopExit => stopLatch=false]
  SH1 --> SH2{still holding?}
  SH2 -- No --> L
  SH2 -- Yes --> SH3[w = headPID.update(-headErr, dt)<br/>if |headErr| < 1° => w=0, wCmd=0<br/>else wCmd += slew(w - wCmd)]
  SH3 --> SH4[tankDrive(wCmd, -wCmd)]
  SH4 --> SH5[stopHold += dt]
  SH5 --> SH6{stopHold >= stopSettleMs?}
  SH6 -- Yes --> X
  SH6 -- No --> L

  %% ---------------- NORMAL DRIVE ----------------
  H -- No --> C[Dynamic speed cap<br/>cap = minCap + 200*|distErr|<br/>cap = min(cap, maxSpeedPct)<br/>distPID output limits = ±cap]

  C --> P[PID outputs<br/>v = distPID.update(currDist, dt)<br/>w = headPID.update(-headErr, dt)]

  P --> W[Heading scaling<br/>wScale = min(1, |v|/18)<br/>wScale = max(wScale, 0.25)<br/>if |v|<12 && |headErr|>2° => wScale=max(wScale,0.22)<br/>w *= wScale]

  W --> SL[Slew limit<br/>vCmd += clamp(v - vCmd, -dvMax, dvMax)<br/>wCmd += clamp(w - wCmd, -dwMax, dwMax)]

  SL --> ZC{Zero-crossing?}
  ZC -- Yes --> ZC1[crossed=true] --> SE
  ZC -- No --> SE

  SE{Enter stopLatch?<br/>!stopLatch && (|distErr| < stopEnter || crossed)}
  SE -- Yes --> SE1[stopLatch=true<br/>vCmd=0, wCmd=0] --> OUT
  SE -- No --> OUT

  OUT[tankDrive<br/>left=vCmd+wCmd<br/>right=vCmd-wCmd] --> SET[Settling<br/>if |distErr|<0.02 && |vCmd|<6 => settled+=dt<br/>else settled=0]

  SET --> DONE{settled >= 40ms?}
  DONE -- Yes --> X
  DONE -- No --> UPD[Update<br/>prevErr=distErr<br/>elapsed += dt<br/>wait(dt)] --> L
