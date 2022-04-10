#include "FRC3484_Lib/components/drive/SC_SwerveDrive.h"
#include "FRC3484_Lib/utils/SC_Constants.h"

using namespace SC;
using namespace std;
using namespace frc;

SC_SwerveDrive::SC_SwerveDrive(SC::SC_SwerveConfig Cfg, units::velocity::meters_per_second_t MaxLinearSpeed, units::velocity::meters_per_second_t MaxAngularSpeed, frc::I2C::Port imuPort)
{
    this->_FL = new SC_SwerveModule(Cfg.FL_cfg, MaxLinearSpeed);
    this->_FR = new SC_SwerveModule(Cfg.FR_cfg, MaxLinearSpeed);
    this->_BL = new SC_SwerveModule(Cfg.BL_cfg, MaxLinearSpeed);
    this->_BR = new SC_SwerveModule(Cfg.BR_cfg, MaxLinearSpeed);

    this->_maxLinSpeed = MaxLinearSpeed.value();
    this->_maxAngSpeed = MaxAngularSpeed.value();
    
    this->_imu = new AHRS(imuPort);
    _imu->Reset();

    this->_sdKinematics = new SwerveDriveKinematics<4>(Cfg.FL_cfg.ModuleLoc, Cfg.FR_cfg.ModuleLoc, Cfg.BL_cfg.ModuleLoc, Cfg.BR_cfg.ModuleLoc);
    this->_sdOdometry = new SwerveDriveOdometry<4>(*_sdKinematics, Rotation2d(0_deg), Pose2d{});

    _cs = ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s};
}

SC_SwerveDrive::~SC_SwerveDrive()
{
    if(_FL != NULL) { delete _FL; }
    if(_FR != NULL) { delete _FR; }
    if(_BL != NULL) { delete _BL; }
    if(_BR != NULL) { delete _BR; }

    if(_imu != NULL) { delete _imu; }
}

void SC_SwerveDrive::Drive(double JS_X, double JS_Y, double JS_Theta)
{
    using units::velocity::meters_per_second_t;
    using units::angular_velocity::radians_per_second_t;

    _cs.vx = units::make_unit<meters_per_second_t>(F_Scale(-1.0, 1.0, -_maxLinSpeed, _maxLinSpeed, JS_X));
    _cs.vy = units::make_unit<meters_per_second_t>(F_Scale(-1.0, 1.0, -_maxLinSpeed, _maxLinSpeed, JS_Y));
    _cs.omega = units::make_unit<radians_per_second_t>(F_Scale(-1.0, 1.0, -_maxAngSpeed, _maxAngSpeed, JS_Theta));

    auto [fl, fr, bl, br] = _sdKinematics->ToSwerveModuleStates(_cs);

    if(_FL != NULL) { _FL->Calculate(SwerveModuleState::Optimize(fl, _FL->GetSteerAngle())); }
    if(_FR != NULL) { _FR->Calculate(SwerveModuleState::Optimize(fr, _FR->GetSteerAngle())); }
    if(_BL != NULL) { _BL->Calculate(SwerveModuleState::Optimize(bl, _BL->GetSteerAngle())); }
    if(_BR != NULL) { _BR->Calculate(SwerveModuleState::Optimize(br, _BR->GetSteerAngle())); } 
}

void SC_SwerveDrive::DriveToPosition(double X_SP, double Y_SP, double Theta_SP)
{

}

void SC_SwerveDrive::AlignWheels()
{
    if(_FL != NULL) { _FL->ReturnToZero(); }
    if(_FR != NULL) { _FR->ReturnToZero(); }
    if(_BL != NULL) { _BL->ReturnToZero(); }
    if(_BR != NULL) { _BR->ReturnToZero(); }
}