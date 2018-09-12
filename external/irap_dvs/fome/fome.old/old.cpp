//this is functions that are not used anymore,
//but which ideas could utilized further.
bool Estimator::estimate_velocity(int x, int y, bool polr)
{ // estimate velocity by bounding region (now 5*5)
	double dx = 0;
	double dy = 0;
	double dt = event_clock_now.toSec() - Eventstack[x][y]->decay_time;
	if (dt < 0) return false;
	for (int i = -2 ; i < 3 ; i++)
		for (int j = -2 ; j < 3 ; j++)
				timedecay(x+i,y+j);
	for (int j = -2 ; j < 3 ; j++)
	{
		for (int i = -2 ; i < 0 ; i++)
			if (Eventstack[x+i][y+j]->update_time.toSec() > dt)
				dx -= abs(Eventstack[x+i][y+j]->eventlevel) / i;
		for (int i = 1 ; i < 3 ; i++)
			if (Eventstack[x+i][y+j]->update_time.toSec() > dt)
				dx -= abs(Eventstack[x+i][y+j]->eventlevel) / i;
	}
	for (int i = -2 ; i < 3 ; i++)
	{
		for (int j = -2 ; j < 0 ; j++)
			if (Eventstack[x+i][y+j]->update_time.toSec() > dt)
				dy += abs(Eventstack[x+i][y+j]->eventlevel) / j;
		for (int j = 1 ; j < 3 ; j++)
			if (Eventstack[x+i][y+j]->update_time.toSec() > dt)
				dy += abs(Eventstack[x+i][y+j]->eventlevel) / j;
	}
	
	if ((dx<0.001 && dy<0.001) || isnan(dx) || isnan(dy)) return false;
	double direction = 57.29578*atan2(dy,dx);
	if (direction < 0)	direction += 360;
	double speed = dx*dx + dy*dy;

	pthread_mutex_lock(&Eventstack[x][y]->mtx);
	Eventstack[x][y]->status = 1;
	Eventstack[x][y]->velocity = {speed, direction};
	Eventstack[x][y]->decay_time = decay_rate;
	pthread_mutex_unlock(&Eventstack[x][y]->mtx);
	return true;
}

cv::Mat cv_imageout;
int key = 0;
cv::Mat rotate(cv::Mat raw_img, double angle)
{
    cv::Mat rotated_img;
    cv::Point2f pt(raw_img.cols/2., raw_img.rows/2.);
    cv::Mat r = getRotationMatrix2D(pt, angle, 1.0);
    cv::warpAffine(raw_img, rotated_img, r, cv::Size(raw_img.rows, raw_img.cols));
    return rotated_img;
}
  //use with this
cv_imageout = rotate(cv_imageout,270);

