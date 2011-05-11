#include "myfuncs.h"

using namespace cv;


//Algebra and Geometry
Mat getCOM(const Mat points){
	Mat x = points.col(0);
	Mat y = points.col(1);
	Mat z = points.col(2);
	Mat com = (Mat_<float>(1,3) << mean(x).val[0], mean(y).val[0], mean(z).val[0]);
	return com;
}


Mat cvCalcXCovarMatrix(Mat pts1,Mat pts2,Mat com1,Mat com2){
	Mat xcovarmat;
	xcovarmat.zeros(3,3,CV_32F);
	Mat pt1;
	Mat pt2;
	for (int i=0;i<pts1.rows;i++){
		pts1.row(i).copyTo(pt1);
		pts2.row(i).copyTo(pt2);
		pt1=pt1-com1;
		pt2=pt2-com2;
		xcovarmat = xcovarmat + pt2.t()*pt1;//cvGEMM(pt2,pt1,1,xcovarmat,1,xcovarmat,CV_GEMM_A_T);
	}
	return xcovarmat;
}
void calcTransformSVD(Mat model, Mat observed, Mat Rotation, Mat Translation){
			//First we get the Center of Mass of each set
			Mat model_com = getCOM(model);
			Mat observ_com = getCOM(observed);
			//Cross-covariance matrix
			Mat W = cvCalcXCovarMatrix(model,observed,model_com,observ_com);
			//Compute the Singular Value decomposition
			Mat U,S,V;
			//CvMat* S = cvCreateMat(3,3,CV_32FC1);
			//CvMat* V = cvCreateMat(3,3,CV_32FC1);
			//CvSVD(&W,S,U,V,CV_SVD_V_T);
			
			//Get Optimal Rotation
			Rotation= SVD(W).u*SVD(W).vt;//cvMatMul(U,V,Rotation);
			//Recover the translation once optimal rotation is found
			Translation = observ_com.t() - Rotation*model_com.t();//cvGEMM(Rotation,model_com,-1,observ_com,1,Translation,CV_GEMM_B_T+CV_GEMM_C_T);
}

void calcTransformHorn(const Mat model,const Mat observed, Mat* Rotation, Mat* Translation) {
			//First we get the Center of Mass of each set
			Mat model_com = getCOM(model);
			Mat observ_com = getCOM(observed);
			//Cross-covariance matrix
			Mat W = cvCalcXCovarMatrix(observed,model,observ_com,model_com);
			
			Mat A = W - W.t();			
			Mat_<float> delta(3,1);
			delta << A.at<float>(1,2), A.at<float>(2,0), A.at<float>(0,1);
			float trW = cv::trace(W).val[0];
			Mat Q(4,4,CV_32F);
			Q(Range(1,4),Range(1,4))=W+W.t()-trW*(Mat::eye(3,3,CV_32F));
			Q.at<float>(0,0)=trW;
			Q(Range(0,1),Range(1,4))=delta.t();
			Q.at<float>(1,0)=delta(0);
			Q.at<float>(2,0)=delta(1);
			Q.at<float>(3,0)=delta(2);
						
			Mat Val, Vec;
			eigen(Q,Val,Vec,0,0);
			Mat_<float> q = Vec.row(0);
			*Rotation = (cv::Mat_<float>(3,3) <<
				pow(q(0,0),2)+pow(q(0,1),2)-pow(q(0,2),2)-pow(q(0,3),2),
				2*(q(0,1)*q(0,2)-q(0,0)*q(0,3)),
				2*(q(0,1)*q(0,3)+q(0,0)*q(0,2)),
				2*(q(0,1)*q(0,2)+q(0,0)*q(0,3)),
				pow(q(0,0),2)+pow(q(0,2),2)-pow(q(0,1),2)-pow(q(0,3),2),
				2*(q(0,2)*q(0,3)-q(0,0)*q(0,1)),
				2*(q(0,1)*q(0,3)-q(0,0)*q(0,2)),
				2*(q(0,2)*q(0,3)+q(0,0)*q(0,1)),
				pow(q(0,0),2)+pow(q(0,3),2)-pow(q(0,1),2)-pow(q(0,2),2));
			
			*Translation = observ_com.t()-(*Rotation)*model_com.t();
}

//Output

void printMatCv(CvMat* matrix){
	printf("\n");
	for (int row = 0;row<matrix->rows;row++){
		const float* ptr = (const float*)(matrix->data.ptr +row*matrix->step);
		printf("|	");
		for (int col=0;col<matrix->cols;col++)
			printf("%f	",*ptr++);
		printf("|\n");
	}
}


void printMat2(const Mat matrix){
	const int chans = matrix.channels();
	printf("\n");
	for (int row = 0;row<matrix.rows;row++){
		printf("|");
		for (int col=0;col<matrix.cols;col++){
			for (int channel=0;channel< chans;channel++){
				if (channel == 0)
					printf(" %f",matrix.at<Vec3f>(row,col)[0]);
				else
					printf(",%f",matrix.at<Vec3f>(row,col)[channel]);
			}
			printf("	");
		}
		printf("|\n");
	}
}

//void depth2pcl(

//Algorithms
void ICP(const Mat model,const Mat observed, Mat Rotation, Mat Translation){
	//FileStorage fs("icp.yml",FileStorage::APPEND);
	BruteForceMatcher<L2<float> > closest_matcher;
	std::vector<DMatch> matches;
	//ICP parameters
	float err_th = 200;
	int iterations = 500;
	Mat R, T,distance,transpt;
	float error,dist;
	float last_error = 0;
	Mat Yk(model.rows,3,CV_32F);
	Mat Pk(model.rows,3,CV_32F);
	Mat tmprow;
	//P0=P
	model.copyTo(Pk);
	for (int it=0;it<iterations;it++){
		//Compute closest points Yk=C(Pk,X)
		closest_matcher.match(Pk,observed,matches);
		for (int i=0;i<matches.size();i++){
			tmprow = Yk.row(matches[i].queryIdx);
			(observed.row(matches[i].trainIdx)).copyTo(tmprow);
		}
		//Calculate transformations qk=Q(P0,Yk)
		calcTransformHorn(model,Yk,&R,&T);
		//Compute mean-square-error and Apply Registration P_(k+1)=qk(P0)
		error = 0;
		for (int p=0;p<matches.size();p++){
				transpt = R*model.row(p).t()+T;
				tmprow = Pk.row(p);
				transpt.copyTo(tmprow);
				distance = Yk.row(p).t() - Pk.row(p).t();
				dist =norm(distance);
				//printMat(distance);
				//printf("match: %d dist =%f\n",p,dist);
				error+=pow(dist,2);
		}
		error = error/matches.size();
		
		//fs << "Iteration" << it <<"error" << error;
		//Terminate when change in m-sq-e < err_th
		if (error-last_error < err_th && error < 1000)
			break;
		last_error = error;
	}
	R.copyTo(Rotation);
	T.copyTo(Translation);
}

int findTransformationRANSAC(const Mat model_pts,const Mat current_pts,int* consensus_set,Mat Rotation, Mat Translation, float error){
		
		assert(model_pts.rows == current_pts.rows);
	
		//RANSAC parameters
		error = FLT_MAX;
		int th = 15;
		int min_inliers = 3;
		int iterations=500;
		const int initpts = 3;
		
		//init variables
		int matches = model_pts.rows;
		int randompt;
		int inliers=0;
		int binliers=0;
		float msq_error=0;
		float dist;
		//Convert points
		Mat init_model(initpts,3,CV_32F);
		Mat init_observ(initpts,3,CV_32F);
		Mat distance;
		Mat transpt;
		Mat R(3,3,CV_32F);
		Mat T(3,1,CV_32F);
		srand((unsigned)time(0));
		Mat tmprow;
		for (int i=0;i<iterations;i++){
			//First we get some number(initpts) of random points
			for (int pt=0;pt<initpts;pt++){
				randompt = rand()%matches;
				tmprow = init_model.row(pt);
				model_pts.row(randompt).copyTo(tmprow);
				tmprow = init_observ.row(pt);
				current_pts.row(randompt).copyTo(tmprow);
				}
			
			//Get the Optimal Transformation
			//calcTransformSVD(init_model,init_observ,R,T);
			calcTransformHorn(init_model,init_observ,&R,&T);
			//add points within a threshold to the consensus_set (inliers)
			inliers=0;
			msq_error=0;
			for (int p=0;p<matches;p++){
				transpt = R*model_pts.row(p).t()+T;
				distance = current_pts.row(p).t() - transpt;
				dist =norm(distance);
				if (dist >= 0 && dist < th){
					consensus_set[inliers]=p;
					inliers++;
					msq_error+=pow(dist,2);
				}
			}
			msq_error=msq_error/inliers;
			if (inliers >= min_inliers) {
				//fine adjustment - error minimization
				Mat r_model(inliers,3,CV_32F);
				Mat r_observ(inliers,3,CV_32F);
				for (int p=0;p<inliers;p++){
					tmprow = r_model.row(p);
					model_pts.row(consensus_set[p]).copyTo(tmprow);
					tmprow = r_observ.row(p);
					current_pts.row(consensus_set[p]).copyTo(tmprow);
				}
				//calcTransformSVD(r_model,r_observ,R,T);
				calcTransformHorn(r_model,r_observ,&R,&T);
	
				msq_error=0;
				int pin=inliers;
				inliers=0;
				for (int p=0;p<pin;p++){
					transpt = R*model_pts.row(p).t()+T;
					distance = current_pts.row(p).t() - transpt;
					dist = norm(distance);
					if (dist>=0 && dist<th){
						consensus_set[inliers]=p;
						inliers++;
						msq_error +=pow(dist,2);
					}
				}
				msq_error=msq_error/inliers;		
				//update best fit
				if (msq_error<error && inliers>min_inliers){
					//recalculate inliers
					binliers=inliers;
					error = msq_error;
					R.copyTo(Rotation);
					T.copyTo(Translation);
					//printf("iteration: %d error: %f inliers:%d\n",i,error,binliers);
				}
			}
			if(error<30) break;
		}
		return binliers;
	}
