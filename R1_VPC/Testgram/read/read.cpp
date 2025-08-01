#include <iostream>
#include <string>
#include <regex>
#include <vector>
#include <iomanip>

using namespace std;

// 提取单个多项式的系数
vector<string> extractCoefficients(const string& expr) {
    vector<string> coeffs;
    
    // 正则表达式匹配各项系数（注意：中文符号和英文符号的区别，这里统一用英文符号）
    regex rx3(R"(([-+]?\d+\.\d+)x?)");  // 匹配x?的系数
    regex rx2(R"(([-+]?\d+\.\d+)x?)");  // 匹配x?的系数
    regex rx1(R"(([-+]?\d+\.\d+)x)");   // 匹配x的系数
    regex rx0(R"(([-+]?\d+\.\d+)$)");   // 匹配常数项

    smatch match;
    
    // 提取x?系数
    if (regex_search(expr, match, rx3)) {
        coeffs.push_back(match[1].str());
    }
    
    // 提取x?系数
    if (regex_search(expr, match, rx2)) {
        coeffs.push_back(match[1].str());
    }
    
    // 提取x系数
    if (regex_search(expr, match, rx1)) {
        coeffs.push_back(match[1].str());
    }
    
    // 提取常数项
    if (regex_search(expr, match, rx0)) {
        coeffs.push_back(match[1].str());
    }
    
    return coeffs;
}

int main() {
    string line;
    vector<vector<string>> allCoeffs;
    
    // 读取输入直到EOF（输入完成后按Ctrl+Z结束输入）
    while (getline(cin, line)) {
        // 查找包含多项式的行
        size_t pos = line.find("f(x) = ");
        if (pos != string::npos) {
            // 提取多项式表达式部分
            string expr = line.substr(pos + 8);
            
            // 处理表达式中的"+ -"为"-"（例如" + -123"转换为" -123"）
            size_t plusMinus;
            while ((plusMinus = expr.find("+ -")) != string::npos) {
                expr.replace(plusMinus, 3, "-");
            }
            
            // 提取系数
            vector<string> coeffs = extractCoefficients(expr);
            if (coeffs.size() == 4) {  // 确保提取到4个系数（三次多项式）
                allCoeffs.push_back(coeffs);
            }
        }
    }
    
    // 按格式输出系数
    for (size_t i = 0; i < allCoeffs.size(); ++i) {
        const auto& coeffs = allCoeffs[i];
        
        // 输出格式控制，确保对齐
        cout << "{";
        for (size_t j = 0; j < coeffs.size(); ++j) {
            // 根据系数正负调整输出宽度，确保对齐
            if (coeffs[j].front() == '-') {
                cout << setw(11) << coeffs[j] << "f";
            } else {
                cout << " " << setw(10) << coeffs[j] << "f";  // 正数前补空格对齐
            }
            if (j != 3) {
                cout << ", ";
            }
        }
        cout << "}";
        
        // 最后一个系数组不输出逗号
        if (i != allCoeffs.size() - 1) {
            cout << ",//";
        } else {
            cout << ";//";
        }
        cout << endl;
        
        // 除了最后一个，其他后面加缩进（保持格式美观）
        if (i != allCoeffs.size() - 1) {
            cout << "    ";
        }
    }
    
    return 0;
}