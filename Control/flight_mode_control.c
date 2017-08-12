#include "flight_mode_control.h"

void flight_mode_update(enum Flight_Mode _mode)
{
		if (_mode != last_flight_mode) {
				last_flight_mode = _mode;
		}
		switch (_mode) {
				case Stabilize:
					break;
				case AltHold:
					break;
				case OneKeyFlip:
					break;
				case Acro:
					break;
				case Loiter:
					break;
				case PosHold:
					break;
				case Auto:
					break;
				default:
					break;
		}
}

void mannual_run()
{

}

void althold_run()
{

}