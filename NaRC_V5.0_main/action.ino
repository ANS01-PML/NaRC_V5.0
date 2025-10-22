void flc(uint16_t SPEED, int8_t pd) {
  led_ON;
  bitSensor = readSensor();

  uint32_t NOW = millis();
  uint32_t interval = NOW - lastTime;
  if (interval >= ee.Ts) {
    int8_t errorSensor[14] = { -26, -22, -18, -14, -10, -5, -1, 1, 5, 10, 14, 18, 22, 26};
    int8_t onSensor[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int8_t sumOnSensor = 0;
    for (int8_t i = 0; i < 14; i++) {
      onSensor[i] = bitRead(bitSensor, i);
      sumOnSensor += onSensor[i];
      errorSensor[i] = errorSensor[i] * onSensor[i];
      error += errorSensor[i];
    }
    error = error / sumOnSensor;

    int16_t rateError = error - lastError;
    lastError = error;

    int16_t moveVal = int((error * ee.kp[pd]) + (rateError * ee.kd[pd]));
    SPEED = map(SPEED, 0, 100, 0, 255);
    int16_t moveLeft = SPEED - moveVal;
    int16_t moveRight = SPEED + moveVal;

    int16_t VMAX = map(ee.Vmax, -100, 100, -255, 255);
    int16_t VMIN = map(ee.Vmin, -100, 100, -255, 255);

    moveLeft = constrain(moveLeft, VMIN, VMAX);
    moveRight = constrain(moveRight, VMIN, VMAX);
    setMotor(moveLeft, moveRight);
    lastTime = NOW;
  }
}

void flr(uint16_t SPEED, int8_t pd) {
  led_ON;
  bitSensor = readSensor();
  uint32_t NOW = millis();
  uint32_t interval = NOW - lastTime;
  if (interval >= ee.Ts) {
    int8_t errorSensor[14] = { -46, -42, -38, -34, -30, -26, -22, -18, -14, -10, -5, -1, 1, 5};
    int8_t onSensor[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int8_t sumOnSensor = 0;
    for (int8_t i = 0; i < 14; i++) {
      onSensor[i] = bitRead(bitSensor, i);
      sumOnSensor += onSensor[i];
      errorSensor[i] = errorSensor[i] * onSensor[i];
      error += errorSensor[i];
    }
    error = error / sumOnSensor;

    int16_t rateError = error - lastError;
    lastError = error;

    int16_t moveVal = int((error * ee.kp[pd]) + (rateError * ee.kd[pd]));
    SPEED = map(SPEED, 0, 100, 0, 255);
    int16_t moveLeft = SPEED - moveVal;
    int16_t moveRight = SPEED + moveVal;

    int16_t VMAX = map(ee.Vmax, -100, 100, -255, 255);
    int16_t VMIN = map(ee.Vmin, -100, 100, -255, 255);

    moveLeft = constrain(moveLeft, VMIN, VMAX);
    moveRight = constrain(moveRight, VMIN, VMAX);
    setMotor(moveLeft, moveRight);
    lastTime = NOW;
  }
}

void fll(uint16_t SPEED, int8_t pd) {
  led_ON;
  bitSensor = readSensor();

  uint32_t NOW = millis();
  uint32_t interval = NOW - lastTime;
  if (interval >= ee.Ts) {
    int8_t errorSensor[14] = { -5, -1, 1, 5, 10, 14, 18, 22, 26, 30, 34, 38, 42, 46};
    int8_t onSensor[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int8_t sumOnSensor = 0;
    for (int8_t i = 0; i < 14; i++) {
      onSensor[i] = bitRead(bitSensor, i);
      sumOnSensor += onSensor[i];
      errorSensor[i] = errorSensor[i] * onSensor[i];
      error += errorSensor[i];
    }
    error = error / sumOnSensor;
    int16_t rateError = error - lastError;
    lastError = error;

    int16_t moveVal = int((error * ee.kp[pd]) + (rateError * ee.kd[pd]));
    SPEED = map(SPEED, 0, 100, 0, 255);
    int16_t moveLeft = SPEED - moveVal;
    int16_t moveRight = SPEED + moveVal;

    int16_t VMAX = map(ee.Vmax, -100, 100, -255, 255);
    int16_t VMIN = map(ee.Vmin, -100, 100, -255, 255);

    moveLeft = constrain(moveLeft, VMIN, VMAX);
    moveRight = constrain(moveRight, VMIN, VMAX);
    setMotor(moveLeft, moveRight);
    lastTime = NOW;
  }
}