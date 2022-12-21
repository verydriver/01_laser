#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

int* Locate_Laser(Mat image) {
	Mat lst = image.col(image.cols / 2).clone();
	Mat_<uchar> m1 = lst;

	int idx_1 = 0, idx_2 = 0, idx_3 = 0;
	int max = 0, temp = 0, j = 0;
	int y_1 = 0, y_2 = 0;
	int dist = 50;

	// find idx_1
	max = 0; temp = 0; j = 0;
	for (j = 0; j < image.rows; j++) {
		temp = m1(j, 0);
		//cout << (int)temp << endl;
		if (temp > max) {
			max = temp;
			idx_1 = j;
		}
	}

	// find idx_2
	y_1 = idx_1 - dist > 0 ? idx_1 - dist : 0;
	y_2 = idx_1 + dist < image.rows ? idx_1 + dist : image.rows;
	for (j = y_1; j < y_2; j++) {
		m1(j, 0) = 0;
	}
	max = 0; temp = 0;
	for (j = 0; j < image.rows; j++) {
		temp = m1(j, 0);
		if (temp > max) {
			max = temp;
			idx_2 = j;
		}
	}

	// find idx_3
	y_1 = idx_2 - dist > 0 ? idx_2 - dist : 0;
	y_2 = idx_2 + dist < image.rows ? idx_2 + dist : image.rows;
	for (j = y_1; j < y_2; j++) {
		m1(j, 0) = 0;
	}
	max = 0; temp = 0;
	for (j = 0; j < image.rows; j++) {
		temp = m1(j, 0);
		if (temp > max) {
			max = temp;
			idx_3 = j;
		}
	}

	int* idx_ = new int[3];
	idx_[0] = idx_1;
	idx_[1] = idx_2;
	idx_[2] = idx_3;

	return idx_;
}

double get_center(Mat img) {
	Scalar mean = cv::mean(img);
	int matMean = round(mean.val[0]);

	double sum1 = 0;
	double sum2 = 0;
	int i = 0; int temp = 0;
	int ind = 0;

	for (i; i < img.rows; i++) {
		temp = img.ptr<uchar>(i)[0];
		if (temp > matMean) {
			sum2 = sum2 + (double)temp;
		}
	}
	sum2 = sum2 / 2;

	i = 0;
	for (i; i < img.rows; i++) {
		temp = img.ptr<uchar>(i)[0];
		if (temp > matMean) {
			sum1 = sum1 + (double)temp;
			ind = i;
			if (sum1 > sum2) {
				break;
			}
		}
	}
	double value = ind + 1 - (sum1 - sum2) / (double)img.at<uchar>(ind, 0);
	return value;
}

int extract_right(int index1, Mat& img, double* laser_position, int gap, int zones_1, int zones_2) {
	int i = 0;
	uchar* ptr = (uchar*)img.data;
	for (i = img.cols / 2; i < img.cols; i++) {
		int max = 0; int temp = 0;
		int j = 0; int m_index_1 = 0; int m_index_2 = 0;
		for (j = index1 - zones_1; j < index1 + zones_1; j++) {
			temp = *(ptr + j * img.cols + i);
			if (temp > max) {
				max = temp;
				m_index_1 = j;
				m_index_2 = j;
			}
			else if (temp == max) {
				m_index_2 = j;
			}
			else {}
		}
		// cout << m_index_1 << ' ' << m_index_2 << endl;
		index1 = (m_index_1 + m_index_2) / 2;

		Mat lst2 = img(Range(index1 - zones_2, index1 + zones_2), Range(i, i + 1));

		double center = get_center(lst2);
		
		laser_position[i] = center + (double)index1 - (double)zones_2;

		double maxvalue = 0, minvalue = 0;
		Point max_ind, min_ind;
		Mat_<uchar> m2 = lst2;
		minMaxLoc(m2, &minvalue, &maxvalue, &min_ind, &max_ind);
		if (maxvalue - minvalue < gap) {
			return i;
		}
	}
	return 0;
}

int extract_left(int index1, Mat& img, double* laser_position, int gap, int zones_1, int zones_2) {
	int k = 0; int i = 0;
	uchar* ptr = (uchar*)img.data;
	for (k = img.cols / 2 + 1; k < img.cols + 1; k++) {
		i = img.cols - k;
		int max = 0; int temp = 0;
		int j = 0; int m_index_1 = 0; int m_index_2 = 0;
		for (j = index1 - zones_1; j < index1 + zones_1; j++) {
			temp = *(ptr + j * img.cols + i);
			if (temp > max) {
				max = temp;
				m_index_1 = j;
				m_index_2 = j;
			}
			else if (temp == max) {
				m_index_2 = j;
			}
		}
		index1 = (m_index_1 + m_index_2) / 2;

		Mat lst2 = img(Range(index1 - zones_2, index1 + zones_2), Range(i, i + 1));

		double center = get_center(lst2);
		laser_position[i] = center + index1 - zones_2;

		double maxvalue = 0, minvalue = 0;
		Point max_ind, min_ind;
		Mat_<uchar> m2 = lst2;
		minMaxLoc(m2, &minvalue, &maxvalue, &min_ind, &max_ind);
		if (maxvalue - minvalue < gap) {
			return i;
		}
	}
	return 0;
}

double sum_(double* lst, int begin, int end) {
	int i = begin;
	double sum = 0;
	for (i; i < end; i++) {
		sum += lst[i];
	}
	return sum;
}

int main() {
	String path = ".\\0.bmp";
	Mat img = imread(path, COLOR_BGR2GRAY);

	clock_t start, end;
	start = clock();

	// Locate_Laser
	double t1 = (double)getTickCount();
	int* idx_ = Locate_Laser(img);
	t1 = ((double)getTickCount() - t1) / getTickFrequency();

	// array storing laser line
	double t2_1 = (double)getTickCount();

	double laser_position_1[4200] = { 0 };
	double laser_position_2[4200] = { 0 };
	double laser_position_3[4200] = { 0 };

	// define parameters
	int gap = 20; int zones_1 = 20; int zones_2 = 20;

	t2_1 = ((double)getTickCount() - t2_1) / getTickFrequency();

	//  extract_right
	double t2_2 = (double)getTickCount();
	int laser_1_right = extract_right(idx_[0], img, laser_position_1, gap, zones_1, zones_2);
	int laser_2_right = extract_right(idx_[1], img, laser_position_2, gap, zones_1, zones_2);
	int laser_3_right = extract_right(idx_[2], img, laser_position_3, gap, zones_1, zones_2);
	t2_2 = ((double)getTickCount() - t2_2) / getTickFrequency();
	
	//  extract_left
	double t2_3 = (double)getTickCount();
	int laser_1_left = extract_left(idx_[0], img, laser_position_1, gap, zones_1, zones_2);
	int laser_2_left = extract_left(idx_[1], img, laser_position_2, gap, zones_1, zones_2);
	int laser_3_left = extract_left(idx_[2], img, laser_position_3, gap, zones_1, zones_2);

	t2_3 = ((double)getTickCount() - t2_3) / getTickFrequency();

	// smooth
	double t3 = (double)getTickCount();

	double laser_position_1_new[4200] = { 0 };
	double laser_position_2_new[4200] = { 0 };
	double laser_position_3_new[4200] = { 0 };
	int size = 20;
	int i = 0;
	for (i = laser_1_left + size; i < laser_1_right - size; i++) {
		laser_position_1_new[i] = sum_(laser_position_1, i - size, i + size) / (size * 2);
	}
	for (i = laser_2_left + size; i < laser_2_right - size; i++) {
		laser_position_2_new[i] = sum_(laser_position_2, i - size, i + size) / (size * 2);
	}
	for (i = laser_3_left + size; i < laser_3_right - size; i++) {
		laser_position_3_new[i] = sum_(laser_position_3, i - size, i + size) / (size * 2);
	}

	t3 = ((double)getTickCount() - t3) / getTickFrequency();

	end = clock();
	cout << "time = " << double(end - start) << "ms" << endl;
	cout << "locate time = " << t1 * 1000 << "ms" << endl;
	cout << "extract time 1 = " << t2_1 * 1000 << "ms" << endl;
	cout << "extract time 2 = " << t2_2 * 1000 << "ms" << endl;
	cout << "extract time 3 = " << t2_3 * 1000 << "ms" << endl;
	cout << "smooth time = " << t3 * 1000 << "ms" << endl;

	// release sourse
	delete[] idx_;

	// result preview
	Mat img_new(img.rows, img.cols, CV_8UC1, Scalar(0));
	for (int i = 0; i < img.cols; i++) {
		if (laser_position_1[i] != 0) {
			img_new.at<uchar>((int)laser_position_1_new[i], i) = 255;
		}
		if (laser_position_2[i] != 0) {
			img_new.at<uchar>((int)laser_position_2_new[i], i) = 255;
		}
		if (laser_position_3[i] != 0) {
			img_new.at<uchar>((int)laser_position_3_new[i], i) = 255;
		}
	}
	imwrite(".\\output.jpeg", img_new);

	// sub pixel result preview
	Size dsize = Size(img.cols, 10 * img.rows);
	Mat img_show;
	resize(img, img_show, dsize, 0, 0, INTER_AREA);
	Mat img_showRGB;
	cvtColor(img_show, img_showRGB, COLOR_GRAY2RGB);

	for (int i = 0; i < 4200; i++) {
		double a = laser_position_1_new[i];
		double b = laser_position_2_new[i];
		double c = laser_position_3_new[i];
		if (laser_position_1[i] != 0) {
			img_showRGB.at<Vec3b>((int)(a * 10), i)[0] = 0;
			img_showRGB.at<Vec3b>((int)(a * 10), i)[1] = 255;
			img_showRGB.at<Vec3b>((int)(a * 10), i)[2] = 0;
		}
		if (laser_position_2[i] != 0) {
			img_showRGB.at<Vec3b>((int)(b * 10), i)[0] = 0;
			img_showRGB.at<Vec3b>((int)(b * 10), i)[1] = 255;
			img_showRGB.at<Vec3b>((int)(b * 10), i)[2] = 0;
		}
		if (laser_position_3[i] != 0) {
			img_showRGB.at<Vec3b>((int)(c * 10), i)[0] = 0;
			img_showRGB.at<Vec3b>((int)(c * 10), i)[1] = 255;
			img_showRGB.at<Vec3b>((int)(c * 10), i)[2] = 0;
		}
	}
	imwrite(".\\output_sub_pixel.jpeg", img_showRGB);

	system("pause");
	return 0;
}
