#include <ADXL335.h>

class PeakDetect {
  private:
    int lag;
    float threshold, influence;
    float *buf;
    int idx = 0;
    bool ready = false;

    float average() {
      float a = 0.0;
      for (int i = 0; i < lag; ++i) {
        a += buf[i];
      }
      return a / lag;
    }

    float std_dev(float avg) {
      float deltas = 0.0;
      float res = 0.0;
      for (int i = 0; i < lag; ++i) {
        deltas += pow(buf[i] - avg, 2);
      }
      res = sqrt(deltas / lag);
      return res;
    }

    int detect(float v) {
      float avg = average();
      float dev = std_dev(avg);
      int result = 0;
      int prevIdx = idx == 0 ? (lag - 1) : (idx - 1);
      if (abs(v - avg) > threshold * dev) {
        result = v > avg ? 1 : -1;
        v = influence * v + (1 - influence) * buf[prevIdx];
      }
      buf[idx] = v;
      idx = (idx + 1) % lag;
      return result;
    }
  public:
    PeakDetect(int lag, float threshold, float influence) :
      lag(lag), threshold(threshold), influence(influence) {
      buf = (float*) malloc(sizeof(float) * lag);
    }

    ~PeakDetect() {
      free(buf);
    }

    int addValue(float v) {
      if (ready) {
        return detect(v);
      }
      else {
        buf[idx] = v;
        idx = (idx + 1) % lag;
        ready |= idx == 0;
        return  0;
      }
    }
};

ADXL335 acc(A4, A3, A2);
PeakDetect pdX(100, 4.5, 0.2);
PeakDetect pdY(100, 4.5, 0.2);
PeakDetect pdZ(100, 4.5, 0.2);

void setup() {
  Serial.begin(9600);

  acc.setExtVoltage(5.13);
  acc.setZeroGVoltage(X, 1.64);
  acc.setZeroGVoltage(Y, 1.64);
  acc.setZeroGVoltage(Z, 1.66);
  acc.setSensitivity(X, 0.3);
  acc.setSensitivity(Y, 0.3);
  acc.setSensitivity(Z, 0.4);
}

void loop() {
  acc.update(20, 1);
  
  float x = acc.getAcceleration(X);
  float y = acc.getAcceleration(Y);
  float z = acc.getAcceleration(Z);

  int xPeak = pdX.addValue(x);
  int yPeak = pdY.addValue(y);
  int zPeak = pdZ.addValue(z);

  Serial.print(x);
  Serial.print("," );
  Serial.print(xPeak * 10);
  Serial.print("," );
  Serial.print(y);
  Serial.print("," );
  Serial.print(yPeak * 10);
  Serial.print("," );
  Serial.print(z);
  Serial.print("," );
  Serial.print(zPeak * 10);
  Serial.println();
  delay(10);
}
