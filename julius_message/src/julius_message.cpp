#include <iostream>
#include <boost/regex.hpp>
#include <boost/asio.hpp>
#include <fstream>
#include <ros/ros.h>
#include <unistd.h>

//  文字列を置換する
std::string Replace( std::string String1, std::string String2, std::string String3 )
{
    std::string::size_type  Pos( String1.find( String2 ) );

    while( Pos != std::string::npos )
    {
        String1.replace( Pos, String2.length(), String3 );
        Pos = String1.find( String2, Pos + String3.length() );
    }

    return String1;
}

int main(int argc, char const* argv[])
{
        using namespace boost::asio;
        using namespace std;
	ofstream fs1("/home/yhirai/docomo/message.txt");

	sleep(3);
        // Julius の ip と port を指定
        const std::string ip = "133.19.23.117";
        const int port       = 10500;

        // Julius へ接続
        ip::address addr = ip::address::from_string(ip);
        ip::tcp::endpoint ep(addr, port);
        ip::tcp::iostream s(ep);

        // Julius サーバ 起動
        s << "STATUS\n" << std::flush;

        // 応答を解析する正規表現
        boost::regex r("WORD=\"([^\"]+)\"");
        boost::smatch m;

        // Julius からの応答を解析
        string line, word;
        while (getline(s, line)) {
                if (boost::regex_search(line, m, r))
		  word += m.str(1);
                else if (line.find("<RECOGOUT>") != string::npos)
		  word = "";
                else if (line.find("</RECOGOUT>") != string::npos){
		  word = Replace(word,"。","");
		  fs1 << word << endl;
		}
        }

        return 0;
}

