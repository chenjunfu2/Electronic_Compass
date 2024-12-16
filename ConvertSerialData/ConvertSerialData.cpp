#include <stdio.h>
#include <cstdint>

int main(void)
{
	FILE *f = fopen("write.bin", "rb");
	if (f == NULL)
	{
		return -1145;
	}

	FILE *fc = fopen("convert2.txt", "wb");
	if (fc == NULL)
	{
		return -1145;
	}

	//跳过前4行输出
	//while (fgetc(f) != '\n');
	//while (fgetc(f) != '\n');
	//while (fgetc(f) != '\n');
	//while (fgetc(f) != '\n');

	//读取二进制数据
#pragma pack(push,1)
	struct
	{
		int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
	}data;

	struct
	{
		uint8_t jx, jy, jz;
	}adj;
#pragma pack(pop)
	//读出校准数据
	fread((void *)&adj, sizeof(adj), 1, f);
	fprintf(fc, "adj:\n%d %d %d\n", adj.jx, adj.jy, adj.jz);

	fprintf(fc, "9motion:\n");
	//循环读出9轴数据
	while (fread((void *)&data, sizeof(data), 1, f) == 1)
	{
		fprintf(fc, "%d %d %d %d %d %d %d %d %d\n", data.ax, data.ay, data.az, data.gx, data.gy, data.gz, data.mx, data.my, data.mz);
	}

	fclose(fc);
	fclose(f);

	return 0;
}