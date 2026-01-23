flowchart TD
  A([turnTo(targetDeg) or turnBy(deltaDeg)]) --> B[Init<br/>turnTo: targetHead=targetDeg, useHeading=true<br/>turnBy: targetRot=rotationDeg()+deltaDeg, useHeading=false<br/>turnPID: SP=0, reset(0)<br/>wCmd=0, rateFilt=0, settled=0<br/>prevHead=headingDeg(), elapsed=0]

  B --> L{{Loop}}
  L --> T{elapsed >= timeoutMs?}
  T -- Yes --> X[Cleanup<br/>stopDrive(brake)] --> END([Return])

  T -- No --> E[Error<br/>if useHeading: err=angleDiff(targetHead, headingDeg())<br/>else: err=targetRot - rotationDeg()]

  E --> R[Angular rate (filtered)<br/>dHead = wrap(headingDeg()-prevHead)<br/>rate = dHead/dt<br/>rateFilt += alpha*(rate-rateFilt)<br/>prevHead=headingDeg()]

  R --> P[w = turnPID.update(-err, dt)]
  P --> SL[wCmd += clamp(w - wCmd, -dwMax, dwMax)]
  SL --> OUT[tankDrive(wCmd, -wCmd)]

  OUT --> SET[Settling<br/>if |err|<1° && |rateFilt|<8°/s => settled+=dt<br/>else settled=0]

  SET --> DONE{settled >= 150ms?}
  DONE -- Yes --> X
  DONE -- No --> UPD[elapsed += dt<br/>wait(dt)] --> L
