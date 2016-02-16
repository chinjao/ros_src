/*
2015.2.27--------------------------------
誰と誰が何を話したかを記録するモジュール

*/

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>

#include <fstream>

#include <humans_msgs/Humans.h>

#include <speech_msgs/Speech.h>
#include <zmq_ros/mecab_amivoice2.h>
#include <speech_msgs/SpeechSrv.h>

using namespace std;

#define MASTER 1


class Place
{
public:
  string name;
  double x;
  double y;
};

class Situation
{
public:
  int okao_id;
  Place place;
  ros::Time time;
  vector<int> people;
};

class WordProp
{
public:
  int freq;
  int num;
  vector<string> part;
  vector<Situation> situations;
};

//map< int, map< string, WordProp > > user_words;

map< string, WordProp > words_wp;


class SpeechRecog
{
private:
  ros::NodeHandle nh;
  ros::ServiceServer word_srv;
  //ros::Publisher recog_pub_;
  boost::mutex mutex_;
 

  typedef message_filters::Subscriber< 
    humans_msgs::Humans > HumansSubscriber; 

    typedef message_filters::Subscriber< 
    zmq_ros::mecab_amivoice2 > SpeechSubscriber; 

  //typedef message_filters::Subscriber< 
  // speech_msgs::Speech > SpeechSubscriber; 

  HumansSubscriber okao_sub;//, speech_sub;
  SpeechSubscriber speech_sub;

  typedef message_filters::sync_policies::ApproximateTime<
    humans_msgs::Humans, zmq_ros::mecab_amivoice2 /*zmq_ros::mecab_amivoice2*/
    > MySyncPolicy;

  message_filters::Synchronizer< MySyncPolicy > sync;

public:
  SpeechRecog() :
    okao_sub( nh, "/humans/RecogInfo", 10 ),
    speech_sub( nh, "mecab_res", 10 ), 
    sync( MySyncPolicy( 100 ), okao_sub, speech_sub )
  {
    sync.registerCallback( boost::bind( &SpeechRecog::callback, this, _1, _2 ) );
    word_srv 
      = nh.advertiseService("word_search",
			    &SpeechRecog::word_search, this);
    //recog_pub_ = 
    //  nh.advertise<humans_msgs::Humans>("/humans/RecogInfo", 1);

    //ファイルからメモリにこれまでの単語情報を書き込む機能

  }
  ~SpeechRecog()
  {
    //メモリからファイルにこれまでの単語情報を書き込む機能
    cout << "file write" << endl;
    stringstream ss;
    map< string, WordProp >::iterator it_wp = words_wp.begin();
    while( it_wp != words_wp.end() )
      {

	ss << "word: " << it_wp->first << ", freq: " << it_wp->second.freq 
	   << ", num: "<< it_wp->second.num; 

	
	//stringstream sit;
	for(int w_id = 0; w_id < it_wp->second.situations.size() ; ++w_id)
	  {
	    ss << ", situation[ " << w_id <<" ]: " <<  it_wp->second.situations[ w_id ].okao_id; 
	  }
	
	ss << endl;

	++it_wp; 
      }

    std::ofstream ofs("/home/yhirai/catkin_ws/src/okao_client/src/people/words.txt");//,std::ios::out | std::ios::app );
    if(ofs.fail())
      {  // if(!fout)でもよい。
        cout << "file not open" << endl;
        return;
      }
    ofs << ss.str() <<endl;
    cout << "write data:" << ss.str() <<endl;


    //
    words_wp.clear();
  }

  void callback(
		const humans_msgs::HumansConstPtr& okao,
		const zmq_ros::mecab_amivoice2ConstPtr& speech
		)
  {
    boost::mutex::scoped_lock lock(mutex_);
    cout << "test"<< endl;
    
    //1.開始
    //2.okao内のトラッキングIDと結びついた名前（OKAO_ID）を調べる
    //3.もし、主人の名前があったら、4へ。なければ1へ（ループ）
    //4.speech内のトラッキングIDを取得
    //5.トラッキングIDとOKAO_IDを照合する
    //6.OKAO_IDに発言を記録する
    //8.主人以外の人物の記録が終わったら、1へ。終わってなければ6へ。
  
    //人物を検出しているかどうか
    cout << "human size: "<< okao->human.size() <<endl;
    if(okao->human.size())
      {
	bool master_is_looking = false;
	
	map<long long, int> tracking_to_okao;
	//発見した人物を記録し、主人を探す
	for(int i = 0; i<okao->human.size(); ++i)
	  {
	    tracking_to_okao[okao->human[i].body.tracking_id] 
	      = okao->human[i].max_okao_id;
	    cout << "now looking okao_id: " << okao->human[i].max_okao_id << endl; 
	    if( okao->human[i].max_okao_id == MASTER )
	      master_is_looking = true;
	  }

	//主人がいるなら記録開始
	if( master_is_looking )
	  {
	    cout<< "now looking master!" <<endl;
	    int speech_okao_id;
	    //string to long long
	    long long speech_tracking_id = boost::lexical_cast<
	      long long>(speech->tracking_ID); 
      	    cout << "now speech tracking_id: " << speech_tracking_id << endl;

	    map< long long, int >::iterator tracking_id_find
	      = tracking_to_okao.find( speech_tracking_id );
	    speech_okao_id = tracking_id_find->second;
     	    cout << "now speech okao_id: " << speech_okao_id << endl;

	    for( int w_id = 0 ; w_id < speech->word.size() ; ++w_id )
	      {
		WordProp wp_tmp;
		wp_tmp.freq = 0;
		wp_tmp.num = 0;

		map< string, WordProp >::iterator it_words_wp
		  = words_wp.find( speech->word[ w_id ] );
		if( it_words_wp != words_wp.end() )
		  {
		    wp_tmp = it_words_wp->second;
		  }
		
		++wp_tmp.freq;
		wp_tmp.num = speech->number;
		Situation st_tmp;
		st_tmp.okao_id = speech_okao_id;
		//すべての人物をpush_backする
		map< long long, int >::iterator it_okao 
		  = tracking_to_okao.begin(); 
		while(it_okao != tracking_to_okao.end() )
		  {
		    st_tmp.people.push_back( it_okao->second ); 
		    ++it_okao;
		  }
		wp_tmp.situations.push_back( st_tmp );

		words_wp[ speech->word[ w_id ] ] = wp_tmp; 
		  
	      }

	  }

      }

  }
  /*
  void callback(
		const humans_msgs::HumansConstPtr& okao,
		const zmq_ros::mecab_amivoice2ConstPtr& speech
		)
  {
    cout << "test"<< endl;

    
    //1.開始
    //2.okao内のトラッキングIDと結びついた名前（OKAO_ID）を調べる
    //3.もし、主人の名前があったら、4へ。なければ1へ（ループ）
    //4.speech内のトラッキングIDを取得
    //5.トラッキングIDとOKAO_IDを照合する
    //6.OKAO_IDに発言を記録する
    //8.主人以外の人物の記録が終わったら、1へ。終わってなければ6へ。
  
    //人物を検出しているかどうか
    if(okao->num)
      {
	bool master_is_looking = false;
	
	map<long long, int> tracking_to_okao;
	//発見した人物を記録し、主人を探す
	for(int i = 0; i<okao->human.size(); ++i)
	  {
	    tracking_to_okao[okao->human[i].body.tracking_id] 
	      = okao->human[i].max_okao_id;
	
	    if( okao->human[i].max_okao_id == MASTER )
	      master_is_looking = true;
	  }

	//主人がいるなら記録開始
	if( master_is_looking )
	  {
	    cout<< "now looking master!" <<endl;
	    int speech_okao_id;
	    //string to long long
	    long long speech_tracking_id = boost::lexical_cast<
	      long long>(speech->tracking_ID); 
	    
	    cout << "now speech tracking_id:" << speech_tracking_id << endl;

	    //tracking_idをもとにしてOKAO_IDを検索する
	    map< long long, int >::iterator tracking_id_find
	      = tracking_to_okao.find( speech_tracking_id );
	    if( tracking_id_find != tracking_to_okao.end() )
	      {
		//OKAO_IDをキーにしたuser_wordsに、発話された単語を保持する
		speech_okao_id = tracking_id_find->second;

		for(int w_id = 0; w_id < speech->word.size() ; ++w_id)
		  {
		    WordProp wp_tmp;
		    //現在注目している単語の検索
		    map< string, WordProp >::iterator user_word_find
		      = user_words[ speech_okao_id ].find( speech->word[w_id] );

		    if( user_word_find != user_words[ speech_okao_id ].end() )
		      {
			//もし過去にその単語が発話された場合,freqを更新しsituationsを追加
			wp_tmp = 
			  user_words[ speech_okao_id ][ user_word_find->first ];		
		      }	
		    ++wp_tmp.freq;
		    wp_tmp.num = speech->number;
		    
		    Situation st_tmp;
		    //すべての人物をpush_backする
		    map< long long, int >::iterator it_okao 
		      = tracking_to_okao.begin(); 
		    while(it_okao != tracking_to_okao.end() )
		      {
			st_tmp.okao_id.push_back( it_okao->second ); 
			++it_okao;
		      }
		    wp_tmp.situations.push_back( st_tmp );

		    //人物が発話した単語を保持する
		    user_words[ speech_okao_id ][ user_word_find->first ] 
		      = wp_tmp;
		  }
	      } 

	  }

      }

  }
  */

  //検索用関数
  bool word_search(speech_msgs::SpeechSrv::Request &req,
		   speech_msgs::SpeechSrv::Response &res)
  {
    cout<<"now search word: "<< req.word << endl;
  
     map< string, WordProp >::iterator it_s
       = words_wp.find( req.word );



     /*
    while( it_s != user_words.end() )
      {    
	//単語があるかどうかを調べる
	map< string, WordProp >::iterator it_w 
	  = it_s->second.find( req.word );

	if( it_w != it_s->second.end() )
	  {
	    //単語があるなら、そのときの頻度などを返す

	    return true;
	  }

	++it_s;
      }
     */

    return false;
  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "speech_recog");
  SpeechRecog SRObject;
  ros::spin();
  return 0;
}
