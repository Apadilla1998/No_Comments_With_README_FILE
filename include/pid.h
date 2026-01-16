#ifndef PID_H
#define PID_H

#include <algorithm>
#include <cmath>

class PID {
public:
    enum class DerivativeMode { OnError, OnMeasurement };

    PID(double kp = 0.0, double ki = 0.0, double kd = 0.0)
        : kP_(kp), kI_(ki), kD_(kd) {}

    void setGains(double kp, double ki, double kd) { kP_ = kp; kI_ = ki; kD_ = kd; }

    void setSetpoint(double sp) { setpoint_ = sp; }
    double getSetpoint() const { return setpoint_; }

    void setOutputLimits(double outMin, double outMax) {
        outMin_ = outMin; outMax_ = outMax;
        if (outMin_ > outMax_) std::swap(outMin_, outMax_);
    }

    void setIntegralLimits(double iMin, double iMax) {
        iMin_ = iMin; iMax_ = iMax;
        if (iMin_ > iMax_) std::swap(iMin_, iMax_);
    }

    void setIntegralZone(double zone) { iZone_ = std::fabs(zone); }
    void setDerivativeFilterTf(double tfSec) { dFilterTf_ = std::max(0.0, tfSec); }
    void setDerivativeMode(DerivativeMode mode) { dMode_ = mode; }
    void setFeedForward(double ff) { feedForward_ = ff; }
    void setAntiWindupTau(double tauSec) { awTau_ = std::max(0.0, tauSec); }
    void setErrorDeadband(double deadband) { errDeadband_ = std::fabs(deadband); }

    double getError() const { return lastError_; }
    double getOutput() const { return lastOutput_; }
    double getIntegral() const { return integral_; }

    void reset(double measurement = 0.0) {
        integral_ = 0.0;
        prevMeas_ = measurement;
        prevError_ = setpoint_ - measurement;
        dFilt_ = 0.0;
        first_ = true;
        lastError_ = 0.0;
        lastOutput_ = 0.0;
    }

    void resetBumpless(double measurement, double holdOutput = 0.0) {
        const double err = setpoint_ - measurement;

        integral_ = 0.0;
        if (std::fabs(kI_) > 1e-12) {
            integral_ = (holdOutput - feedForward_ - (kP_ * err)) / kI_;
            integral_ = clamp(integral_, iMin_, iMax_);
        }

        prevMeas_ = measurement;
        prevError_ = err;
        dFilt_ = 0.0;
        first_ = true;
        lastError_ = err;
        lastOutput_ = clamp(holdOutput, outMin_, outMax_);
    }

    double update(double measurement, double dtSec) {
        dtSec = std::max(dtSec, 1e-6);

        double error = setpoint_ - measurement;
        if (errDeadband_ > 0.0 && std::fabs(error) < errDeadband_) error = 0.0;
        lastError_ = error;

        double dSignal = 0.0;
        if (!first_) {
            if (dMode_ == DerivativeMode::OnMeasurement) {
                const double dMeas = (measurement - prevMeas_) / dtSec;
                dSignal = -dMeas;
            } else {
                const double dErr = (error - prevError_) / dtSec;
                dSignal = dErr;
            }
        }

        if (first_) {
            dFilt_ = 0.0;
            first_ = false;
        } else if (dFilterTf_ > 0.0) {
            const double alpha = dtSec / (dFilterTf_ + dtSec);
            dFilt_ += alpha * (dSignal - dFilt_);
        } else {
            dFilt_ = dSignal;
        }

        const bool canIntegrate = (std::fabs(kI_) > 1e-12);
        if (canIntegrate && (iZone_ == 0.0 || std::fabs(error) < iZone_)) {
            integral_ += error * dtSec;
            integral_ = clamp(integral_, iMin_, iMax_);
        }

        const double P = kP_ * error;
        const double I = kI_ * integral_;
        const double D = kD_ * dFilt_;
        const double outUnsat = feedForward_ + P + I + D;
        const double outSat = clamp(outUnsat, outMin_, outMax_);

        if (canIntegrate && awTau_ > 0.0) {
            const double delta = (outSat - outUnsat);
            integral_ += (delta / kI_) * (dtSec / awTau_);
            integral_ = clamp(integral_, iMin_, iMax_);
        }

        prevMeas_ = measurement;
        prevError_ = error;
        lastOutput_ = outSat;
        return outSat;
    }

private:
    double kP_ = 0.0, kI_ = 0.0, kD_ = 0.0;

    double setpoint_ = 0.0;
    double feedForward_ = 0.0;

    double outMin_ = -100.0, outMax_ = 100.0;
    double iMin_ = -1e9, iMax_ = 1e9;
    double iZone_ = 0.0;

    DerivativeMode dMode_ = DerivativeMode::OnMeasurement;
    double dFilterTf_ = 0.0;
    double dFilt_ = 0.0;

    double awTau_ = 0.2;
    double errDeadband_ = 0.0;

    double integral_ = 0.0;
    double prevMeas_ = 0.0;
    double prevError_ = 0.0;
    bool first_ = true;

    double lastError_ = 0.0;
    double lastOutput_ = 0.0;

    static double clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }
};

#endif