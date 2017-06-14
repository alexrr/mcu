/*
 * DateTime.c
 *
 *  Created on: 22 мая 2017 г.
 *      Author: alexr
 */
#include "DateTime.h"

void DeltaTime(RTC_TimeTypeDef* dtime_o,RTC_TimeTypeDef* dtime_n,RTC_TimeTypeDef* dtime_d){
	int dsec = dtime_n->Seconds - dtime_o->Seconds;
	int dmin = dtime_n->Minutes - dtime_o->Minutes;
	int dhour = dtime_n->Hours - dtime_o->Hours;
	if(dsec<0){
		dsec+=60;
		--dmin;
	}
	if(dmin<0){
		dmin+=60;
		--dhour;
	}
	if(dhour<0){
		dhour+=24;
	}
	dtime_d->Seconds = dsec;
	dtime_d->Minutes = dmin;
	dtime_d->Hours = dhour;
}
