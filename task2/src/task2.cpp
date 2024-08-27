#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iomanip>

using namespace std;

class ImageMatrix
{
public:
    ImageMatrix(int W = 0, int H = 0) : width(W), height(H), pixels(vector<vector<int>>(H, vector<int>(W))) {} // 创建图像

    bool isEmpty() const; // 判空

    void loadFromFile(const string &filename);     // 读取
    void saveToFile(const string &filename) const; // 保存

    void display() const; // 显示

    void displayBinarized() const; // 二值化显示

    void setPixel(int X, int Y, int VALUE); // 绘制点

    void drawBox(int X, int Y, int W, int H, int VALUE); // 绘制框

    void threshold(int THR); // 阈值化

    void flipHorizontally(); // 翻转

    void rotateClockwise(); // 旋转

private:
    int width, height;
    vector<vector<int>> pixels;
};

// 判空
bool ImageMatrix::isEmpty() const
{
    return width == 0 || height == 0;
}

// 读取
void ImageMatrix::loadFromFile(const string &filename)
{

    ifstream file(filename); // 创建流对象

    if (file.is_open())
    {
        pixels.clear(); // 清除现有的像素数据

        string line;

        while (getline(file, line))
        { // 读取数据并存储到变量line中

            istringstream iss(line);
            vector<int> row;

            int value;

            while (iss >> value)
            { // 使用iss从字符串中提取整数value
                row.push_back(value);
            }

            pixels.push_back(row); // 将row向量添加到pixels二维向量中
        }

        width = pixels.empty() ? 0 : pixels[0].size(); // 更新宽度和高度
        height = pixels.size();

        file.close();
    }
}

// 保存
void ImageMatrix::saveToFile(const string &filename) const
{

    ofstream file(filename);

    if (file.is_open())
    {
        for (const auto &row : pixels)
        {
            for (int pixel : row)
            { // 遍历每一行的的像素值
                file << pixel << ' ';
            }

            file << '\n';
        }

        file.close();
    }
}

// 显示
void ImageMatrix::display() const
{
    for (const auto &row : pixels)
    {
        for (int pixel : row)
        { // 遍历每一行的的像素值
            cout << setw(3) << pixel << "    ";
        }
        cout << '\n';
    }
}

// 二值化显示
void ImageMatrix::displayBinarized() const
{
    for (const auto &row : pixels)
    {
        for (int pixel : row)
        { // 遍历每一行的的像素值
            cout << setw(3) << (pixel == 0 ? '.' : 'O') << "    ";
        }
        cout << '\n';
    }
}

// 绘制点
void ImageMatrix::setPixel(int X, int Y, int VALUE)
{
    if (X >= 0 && X < width && Y >= 0 && Y < height)
    {
        pixels[Y][X] = VALUE;
    }
}

// 绘制框
void ImageMatrix::drawBox(int X, int Y, int W, int H, int VALUE)
{
    for (int i = X; i < X + W; ++i)
    {
        setPixel(i, Y, VALUE);
        setPixel(i, Y + H - 1, VALUE);
    }
    for (int j = Y + 1; j < Y + H - 1; ++j)
    {
        setPixel(X, j, VALUE);
        setPixel(X + W - 1, j, VALUE);
    }
}

// 阈值化
void ImageMatrix::threshold(int THR)
{
    for (auto &row : pixels)
    {
        for (int &pixel : row)
        {
            pixel = (pixel > THR) ? pixel : 0;
        }
    }
}

// 翻转
void ImageMatrix::flipHorizontally()
{
    for (auto &row : pixels)
    {
        reverse(row.begin(), row.end());
    }
}

// 旋转
void ImageMatrix::rotateClockwise()
{
    vector<vector<int>> rotated(width, vector<int>(height)); // 创建新的二维向量rotated，宽度和高度与原图像互换

    for (int i = 0; i < width; ++i)
    {
        for (int j = 0; j < height; ++j)
        {
            rotated[i][height - j - 1] = pixels[j][i]; // 将原图像的第j行第i列变为新图像的第i行第(height-j-1)列
        }
    }
    swap(width, height); // 交换宽度和高度

    pixels = rotated; // 更新像素数据
}

int main()
{
    ImageMatrix img1(5, 7);
    img1.drawBox(0, 0, 5, 7, 255);
    img1.setPixel(0, 0, 0);

    ImageMatrix img2;

    int num;
    cout << "请输入数字" << endl;
    cout << "1.创建图像与绘制" << endl;
    cout << "2.方法调用与组合" << endl;
    cout << "3.综合应用" << endl;

    cin >> num;

    switch (num)
    {
    case 1:
        img1.display();
        break;

    case 2:
        img1.rotateClockwise();
        img1.rotateClockwise();

        img1.flipHorizontally();

        img1.display();

        break;

    case 3:

        img2.loadFromFile("../data.txt");

        if (!img2.isEmpty())
        {
            img2.rotateClockwise();

            img2.threshold(100);
            img2.flipHorizontally();
            img2.displayBinarized();
        }
        else
        {
            cout << "文件为空或路径错误\n";
        }
        break;

    default:
        break;
    }

    return 0;
}