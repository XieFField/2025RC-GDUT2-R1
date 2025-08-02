#include <iostream>
#include <string>
#include <regex>
#include <vector>
#include <iomanip>

using namespace std;

// ��ȡ��������ʽ��ϵ��
vector<string> extractCoefficients(const string& expr) {
    vector<string> coeffs;
    
    // ������ʽƥ�����ϵ����ע�⣺���ķ��ź�Ӣ�ķ��ŵ���������ͳһ��Ӣ�ķ��ţ�
    regex rx3(R"(([-+]?\d+\.\d+)x?)");  // ƥ��x?��ϵ��
    regex rx2(R"(([-+]?\d+\.\d+)x?)");  // ƥ��x?��ϵ��
    regex rx1(R"(([-+]?\d+\.\d+)x)");   // ƥ��x��ϵ��
    regex rx0(R"(([-+]?\d+\.\d+)$)");   // ƥ�䳣����

    smatch match;
    
    // ��ȡx?ϵ��
    if (regex_search(expr, match, rx3)) {
        coeffs.push_back(match[1].str());
    }
    
    // ��ȡx?ϵ��
    if (regex_search(expr, match, rx2)) {
        coeffs.push_back(match[1].str());
    }
    
    // ��ȡxϵ��
    if (regex_search(expr, match, rx1)) {
        coeffs.push_back(match[1].str());
    }
    
    // ��ȡ������
    if (regex_search(expr, match, rx0)) {
        coeffs.push_back(match[1].str());
    }
    
    return coeffs;
}

int main() {
    string line;
    vector<vector<string>> allCoeffs;
    
    // ��ȡ����ֱ��EOF��������ɺ�Ctrl+Z�������룩
    while (getline(cin, line)) {
        // ���Ұ�������ʽ����
        size_t pos = line.find("f(x) = ");
        if (pos != string::npos) {
            // ��ȡ����ʽ���ʽ����
            string expr = line.substr(pos + 8);
            
            // ������ʽ�е�"+ -"Ϊ"-"������" + -123"ת��Ϊ" -123"��
            size_t plusMinus;
            while ((plusMinus = expr.find("+ -")) != string::npos) {
                expr.replace(plusMinus, 3, "-");
            }
            
            // ��ȡϵ��
            vector<string> coeffs = extractCoefficients(expr);
            if (coeffs.size() == 4) {  // ȷ����ȡ��4��ϵ�������ζ���ʽ��
                allCoeffs.push_back(coeffs);
            }
        }
    }
    
    // ����ʽ���ϵ��
    for (size_t i = 0; i < allCoeffs.size(); ++i) {
        const auto& coeffs = allCoeffs[i];
        
        // �����ʽ���ƣ�ȷ������
        cout << "{";
        for (size_t j = 0; j < coeffs.size(); ++j) {
            // ����ϵ���������������ȣ�ȷ������
            if (coeffs[j].front() == '-') {
                cout << setw(11) << coeffs[j] << "f";
            } else {
                cout << " " << setw(10) << coeffs[j] << "f";  // ����ǰ���ո����
            }
            if (j != 3) {
                cout << ", ";
            }
        }
        cout << "}";
        
        // ���һ��ϵ���鲻�������
        if (i != allCoeffs.size() - 1) {
            cout << ",//";
        } else {
            cout << ";//";
        }
        cout << endl;
        
        // �������һ����������������������ָ�ʽ���ۣ�
        if (i != allCoeffs.size() - 1) {
            cout << "    ";
        }
    }
    
    return 0;
}