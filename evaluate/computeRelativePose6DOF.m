function relativePose = computeRelativePose6DOF(previous, current)

Rcw = previous(1:3,1:3);
tcw = previous(1:3,4);
Rwc = Rcw';
Ow = -Rwc*tcw;
inv_pose = eye(4);
inv_pose(1:3,1:3) = Rwc;
inv_pose(1:3,4) = Ow;


relativePose = inv_pose * current;


%     cv::Mat Rcw = Tcw.rowRange(0,2).colRange(0,2);
%     cv::Mat tcw = Tcw.rowRange(0,2).col(2);
%     cv::Mat Rwc = Rcw.t();
%     cv::Mat Ow = -Rwc*tcw;
% 
%     Twc = cv::Mat::eye(3,3,Tcw.type());
%     Rwc.copyTo(Twc.rowRange(0,2).colRange(0,2));
%     Ow.copyTo(Twc.rowRange(0,2).col(2));


end