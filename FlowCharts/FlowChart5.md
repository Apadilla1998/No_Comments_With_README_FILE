flowchart TD
  A([turnTo(targetDeg, timeoutMs)<br/>or turnBy(deltaDeg, timeoutMs)]) --> B[Initialization<br/><br/>turnTo: targetHead=targetDeg, useHeading=true<br/>turnBy: targetRot = rotationDeg()+deltaDeg, useHeading=false<br/><br/>turnPID.setSetpoint(0)<br/>turnPID.reset(0)<br/>wCmd=0, settledMs=0<br/>rateFilt=0, prevHead=currentHeading]

  B --> C{Timeout?<br/>elapsedMs >= timeoutMs}
  C -- Yes --> Z[Cleanup<br/>stopDrive(brake)] --> END([Return])
  C -- No --> D[Read Sensors + Error<br/>if useHeading:<br/>  currHead=headingDeg<br/>  err=angleDiff(targetHead, currHead)<br/>else:<br/>  currRot=rotationDeg<br/>  err=targetRot - currRot]

  D --> E[Compute Angular Rate<br/>dHead = currHead - prevHead<br/>wrap: if dHead>180 => dHead-=360<br/>wrap: if dHead<-180 => dHead+=360<br/>rate = dHead/dt<br/>LPF: rateFilt += alpha*(rate-rateFilt)<br/>prevHead=currHead]

  E --> F[PID Output<br/>w = turnPID.update(-err, dt)]
  F --> G[Slew Limit<br/>dwMax = dwPerSec*dt<br/>wCmd += clamp(w - wCmd, -dwMax, dwMax)]
  G --> H[Apply Output (pure rotation)<br/>tankDrive(wCmd, -wCmd)]

  H --> I[Settling Check<br/>if |err|<1° && |rateFilt|<8°/s<br/>  settledMs += dt<br/>else settledMs = 0]

  I --> J{settledMs >= 150ms?}
  J -- Yes --> Z
  J -- No --> K[Update Timer<br/>elapsedMs += dt<br/>wait(dt)] --> C
