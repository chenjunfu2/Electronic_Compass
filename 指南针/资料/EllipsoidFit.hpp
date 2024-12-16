#pragma once
/*

%�������
%�βΣ�xyzΪҪ��ϵ���ά�������ݣ���������������, sampleΪNx3
%����ֵ��Ox, Oy, OzΪ��ϳ����������ģ�Rx, Ry, RzΪ��������ĳ���

% x = sample(:, 1);
% y = sample(:, 2);
% z = sample(:, 3);

function [VOX, VOY, VOZ, VRX, VRY, VRZ] = ellipsoidFit(x,y,z)

K = [y.^2, z.^2, x, y, z, ones(length(x), 1)];
Y = -x.^2;
V=(K'*K)^(-1)*K'*Y;

VOX = -V(3)/2;% -C/2
VOY = -V(4)/V(1)/2;% -D/A/2
VOZ = -V(5)/V(2)/2;% -E/B/2
VRX = sqrt(VOX^2 + V(1)*VOY^2 + V(2)*VOZ^2 - V(6));% sqrt(VOX^2 + A*VOY^2 + B*VOZ^2 - F)
VRY = sqrt(VRX^2/V(1));% sqrt(VRX^2/A);
VRZ = sqrt(VRX^2/V(2));% sqrt(VRX^2/B);
end

*/

/*
#include <Eigen/Dense>
#include <vector>
#include <cmath>

void ellipsoidFit(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z,
				  double& VOX, double& VOY, double& VOZ, double& VRX, double& VRY, double& VRZ) {
	int n = x.size();

	Eigen::MatrixXd K(n, 6);
	Eigen::VectorXd Y(n);

	for (int i = 0; i < n; ++i) {
		K(i, 0) = y[i] * y[i];
		K(i, 1) = z[i] * z[i];
		K(i, 2) = x[i];
		K(i, 3) = y[i];
		K(i, 4) = z[i];
		K(i, 5) = 1.0;
		Y(i) = -x[i] * x[i];
	}

	Eigen::VectorXd V = (K.transpose() * K).inverse() * K.transpose() * Y;

	VOX = -V(2) / 2.0;
	VOY = -V(3) / V(0) / 2.0;
	VOZ = -V(4) / V(1) / 2.0;
	VRX = std::sqrt(VOX * VOX + V(0) * VOY * VOY + V(1) * VOZ * VOZ - V(5));
	VRY = std::sqrt(VRX * VRX / V(0));
	VRZ = std::sqrt(VRX * VRX / V(1));
}

int main() {
	// ʾ������
	std::vector<double> x = {1.0, 2.0, 3.0};
	std::vector<double> y = {4.0, 5.0, 6.0};
	std::vector<double> z = {7.0, 8.0, 9.0};

	double VOX, VOY, VOZ, VRX, VRY, VRZ;

	ellipsoidFit(x, y, z, VOX, VOY, VOZ, VRX, VRY, VRZ);

	std::cout << "VOX: " << VOX << "\n";
	std::cout << "VOY: " << VOY << "\n";
	std::cout << "VOZ: " << VOZ << "\n";
	std::cout << "VRX: " << VRX << "\n";
	std::cout << "VRY: " << VRY << "\n";
	std::cout << "VRZ: " << VRZ << "\n";

	return 0;
}



********************************************
#include <iostream>
#include <iomanip>
#include <stdexcept>

// ��ӡ����ĺ��������ڵ���
void printMatrix(double matrix[3][3]) {
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			std::cout << std::setw(10) << matrix[i][j] << " ";
		}
		std::cout << std::endl;
	}
}

// ʹ�ø�˹-Լ����Ԫ������3x3����������
bool inverseMatrix(double matrix[3][3], double inverse[3][3]) {
	double augmented[3][6];

	// ����������� [matrix | I]
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			augmented[i][j] = matrix[i][j];
			augmented[i][j + 3] = (i == j) ? 1.0 : 0.0;
		}
	}

	// ���и�˹-Լ����Ԫ
	for (int i = 0; i < 3; ++i) {
		// Ѱ����Ԫ
		double pivot = augmented[i][i];
		if (pivot == 0.0) {
			return false; // ���󲻿���
		}

		// ��һ����Ԫ��
		for (int j = 0; j < 6; ++j) {
			augmented[i][j] /= pivot;
		}

		// ��ȥ�����еĵ�ǰ��
		for (int k = 0; k < 3; ++k) {
			if (k != i) {
				double factor = augmented[k][i];
				for (int j = 0; j < 6; ++j) {
					augmented[k][j] -= factor * augmented[i][j];
				}
			}
		}
	}

	// ��ȡ�����
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			inverse[i][j] = augmented[i][j + 3];
		}
	}

	return true;
}

int main() {
	double matrix[3][3] = {
		{1, 2, 3},
		{0, 1, 4},
		{5, 6, 0}
	};

	double inverse[3][3];

	if (inverseMatrix(matrix, inverse)) {
		std::cout << "Inverse Matrix: " << std::endl;
		printMatrix(inverse);
	} else {
		std::cerr << "Matrix is singular and cannot be inverted." << std::endl;
	}

	return 0;
}




*/

#include <math.h>
//#include <ArduinoEigenSparse.h>
#include <ArduinoEigenDense.h>
//#include <ArduinoEigen.h>

struct Ellipsoid
{
	long double VOX, VOY, VOZ, VRX, VRY, VRZ;
};

template <typename TYPE, size_t COUNT>
Ellipsoid EllipsoidFit(TYPE(&x)[COUNT], TYPE(&y)[COUNT], TYPE(&z)[COUNT])
{
	static Eigen::Matrix<TYPE, COUNT, 6> K;
	static Eigen::Vector<TYPE, COUNT> Y;
	
	for (int i = 0; i < COUNT; ++i)
	{
		K(i, 0) = y[i] * y[i];
		K(i, 1) = z[i] * z[i];
		K(i, 2) = x[i];
		K(i, 3) = y[i];
		K(i, 4) = z[i];
		K(i, 5) = 1.0;
		Y(i) = -x[i] * x[i];
	}

	
	//Eigen::Matrix<TYPE, 6, COUNT> KT = K.transpose();
	static Eigen::Matrix<TYPE, 6, 6> KTI = K.transpose() * K;
	KTI = KTI.inverse();
	static Eigen::Vector<TYPE, 6> KTY = K.transpose() * Y;
	static Eigen::Vector<TYPE, 6> V = KTI * KTY;
	
	Serial.printf("data=%zu,", sizeof(x) + sizeof(y) + sizeof(z));
	Serial.printf("size=%zu\n", sizeof(K) + sizeof(Y) + sizeof(KTI) + sizeof(KTY) + sizeof(V));

	Serial.printf("V=[");
	for (int i = 0; i < 6; ++i)
	{
		Serial.printf("%lf,", V(i));
	}
	Serial.printf("]\n");
	
	Ellipsoid eRet;
	eRet.VOX = -V(2) / 2.0;
	eRet.VOY = -V(3) / V(0) / 2.0;
	eRet.VOZ = -V(4) / V(1) / 2.0;
	eRet.VRX = sqrt(eRet.VOX * eRet.VOX + V(0) * eRet.VOY * eRet.VOY + V(1) * eRet.VOZ * eRet.VOZ - V(5));
	eRet.VRY = sqrt(eRet.VRX * eRet.VRX / V(0));
	eRet.VRZ = sqrt(eRet.VRX * eRet.VRX / V(1));
	
	return eRet;

	//return Ellipsoid{ 101.09 ,-2.43 ,234.65 ,269.56 ,283.29 ,242.96 , };
}


/*
while true
	data0=read(s,3,"int16");%���ٶȼ�
	data1=read(s,3,"int16");%������
	data2=read(s,3,"int16");%������

	ax=data0(1);
	ay=data0(2);
	az=data0(3);

	%ˮƽ����
	roll=atan2(ay,az);
	pitch=atan2(ax,az);

	%roll=atan2(ay,sqrt(ax*ax+az*az));
	%pitch=atan2(ax,sqrt(ay*ay+az*az));

	fprintf("roll=%f,pitch=%f,",roll*180/pi,pitch*180/pi);


	mx=data2(1);
	my=data2(2);
	mz=data2(3);

	%����������У׼
	mx=mx*((((adj(1)-128)*0.5)/128)+1);
	my=my*((((adj(2)-128)*0.5)/128)+1);
	mz=mz*((((adj(3)-128)*0.5)/128)+1);


	%У׼ֵ
	VOX=101.09;
	VOY=-2.43;
	VOZ=234.65;

	VRX=269.56;
	VRY=283.29;
	VRZ=242.96;

	%vk=(VRX+VRY+VRZ)/3;
	vk=1;

	%����У׼
	mx=(mx-VOX)/VRX*vk;
	my=(my-VOY)/VRY*vk;
	mz=(mz-VOZ)/VRZ*vk;

	%plot3(mx,my,mz, '.b');%����
	fprintf("mx=%f,my=%f,mz=%f,",mx,my,mz);

	%��бУ׼
	hy=mx*cos(pitch)+my*sin(roll)*sin(pitch)-mz*cos(roll)*sin(pitch);
	hx=my*cos(roll)+mz*sin(roll);

	%�����뱱ƫ����
	x=atan2(hy,hx);

	fprintf("hy=%f,hx=%f,x=%f\n",hy,hx,x*180/pi);

	%plot(x,'.b');


	%plot3(data0(1),data0(2),data0(3), '.b');%����

	%data2=data2.*((((adj-128).*0.5)./128)+1);%��3�����У׼
	%%data2(3)=-data2(3);%��Z�ᷭת
	%plot3(data2(1),data2(2),data2(3), '.b');%����
end
*/