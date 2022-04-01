#include "FRC3484_Lib/components/drive/SC_SwerveDrive.h"
#include "FRC3484_Lib/utils/SC_Constants.h"

using namespace SC;
using namespace std;
using namespace frc;

SC_SwerveDrive::SC_SwerveDrive(SC::SC_SwerveConfig FL_Cfg, SC::SC_SwerveConfig FR_Cfg, SC::SC_SwerveConfig BL_Cfg, SC::SC_SwerveConfig BR_Cfg, frc::I2C::Port imuPort)
{
    this->_FL = new SC_SwerveModule(FL_Cfg);
    this->_FR = new SC_SwerveModule(FR_Cfg);
    this->_BL = new SC_SwerveModule(BL_Cfg);
    this->_BR = new SC_SwerveModule(BR_Cfg);
    
    this->_imu = new AHRS(imuPort);
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

}

void SC_SwerveDrive::DriveToPosition(double X_SP, double Y_SP, double Theta_SP)
{

}