#include <iostream>
#include <vector>
#include <cmath>
#include "rui-equation-cal.h"
#include "rui-vehicle-beta.h"
using namespace std;

NS_LOG_COMPONENT_DEFINE ("rui-equation-cal");

DataRecoveryHelper::DataRecoveryHelper(void)
  :N(),
  M(),
  s(),
  m(),
  loc()
{
}

/*DataRecoveryHelper::DataRecoveryHelper(int num_v, int num_f)
  :N(num_v),
  M(num_f),
  s(vector<double>(N)),
  m(vector<vector<double> >(M, vector<double>(N+1))),
  loc(vector<int>(M,-1))
{
}*/

void DataRecoveryHelper::SetParameters(int num_v, int num_f, vector<vector<double> > coe)
{
    N = num_v;
    M = num_f;
    s = vector<double>(N);
    loc = vector<int>(M,-100); 
    m = coe;
}
vector<double> DataRecoveryHelper::GetResults()
{
    return s;
}

void DataRecoveryHelper::make1(int h, int n)
{
	double t = m[h][n];
	if (m[h][n] == 0)
		return;
	for (int i = 0; i < N + 1; i++)
		m[h][i] /= t;
}
void DataRecoveryHelper::setloc(int h, int n)
{
	loc[h] = n; 
}
void DataRecoveryHelper::Allmake1(int n)
{
	for (int i = 0; i < M; i++)
	{
		if (loc[i] == -100 && m[i][n] != 0)
			make1(i, n);
	}
}
void DataRecoveryHelper::Onereplace(int n)
{
	int l = 0;
	for (int i = 0; i < M; i++)
	{
		if (loc[i] == -100 && m[i][n] != 0)
		{
			l = i;
			setloc(i, n);
			make1(i, n);
			break;
		}	
	}
	Allmake1(n);
	Allmin(l,n);
}

void DataRecoveryHelper::min(int s,int h,int n)
{
	if(loc[h]==-100&&m[h][n]!=0)
		for (int i = 0; i < N+1; i++)
			m[h][i] -= m[s][i];
}
void DataRecoveryHelper::Allmin(int s,int n)
{
	for (int i = 0; i < M; i++)
		min(s, i, n);
}
void DataRecoveryHelper::Allreplace()
{
	for (int i = 0; i < N; i++)
		Onereplace(i);
}
void DataRecoveryHelper::cacular(int n)
{
	for(int i=0;i<M;i++)
		if (loc[i] == n)
		{
			for (int k = 0; k < N; k++)
				if (m[i][k]!=0&&k != n && s[k]!=0)
				{					
					m[i][N] -= m[i][k] * s[k];
					m[i][k]= 0; 
				}
			int tag = 0;
			for (int k = 0; k < N; k++)
			{
				if (k!=n && m[i][k]!=0)
					tag = 1; //no answer 
			}
			if (tag == 0)
				s[n] = m[i][N] / m[i][n];
			break;
		}
}
void DataRecoveryHelper::Allcacular()
{
	for (int i = N - 1; i >= 0; i--)
		cacular(i);
}
void DataRecoveryHelper::pc()
{
	//fill(loc, loc + M, -1);
	Allreplace();
	Allcacular();
}


vector<string> split(const string &str, const string &pattern)
{
    vector<string> res;
    if(str == "")
        return res;

    string strs = str + pattern;
    size_t pos = strs.find(pattern);

    while(pos != strs.npos)
    {
        string temp = strs.substr(0, pos);
        res.push_back(temp);

        strs = strs.substr(pos+1, strs.size());
        pos = strs.find(pattern);
    }

    return res;
}


DataManagementHelper::DataManagementHelper(int num_obser)//num_obser_expected = group_size-1 = center+others
  :
  num_obser_expected(num_obser),
  obser_list(vector<double>(num_obser+1, -100)),
  obser_list_instance(vector<vector<double>>(num_obser+1, vector<double>(num_entries, -100))), 
  coef(),
  coef_instance(),
  vehicle_id_can_calculate(),
  vehicle_id_can_calculate_instance()
{
  coef_instance.resize(num_entries);
  vehicle_id_can_calculate_instance.resize(num_entries);
  
}

vector<double> DataManagementHelper::GetObserList()
{
	return obser_list;
}

vector <vector<double>> DataManagementHelper::GetObserListInstance()
{
  return obser_list_instance;
}

void DataManagementHelper::AddObserList (double ob_value, int vehicle_id)
{
	obser_list[vehicle_id] = ob_value;
}

void DataManagementHelper::AddObserListInstance (double ob_value, int vehicle_id)
{
  obser_list_instance[vehicle_id].push_back(ob_value);
}

void DataManagementHelper::MessageHandle (string s, int vehicle_id)
{
	vector<string> ss = split(s, "|");
	if (ss.size() == 1)//original packet
	{
		AddObserList(stod(ss[0]), vehicle_id);
	}else if (ss.size() == 2)//forwarded by others
	{
		vector<double> one_function(num_obser_expected+2,0);
		vector<string> passed_vehicles = split(ss[0], "+");
    //passed-vehicles, ID to index
    int node_index = 0;
     

		for (int i=0; i<passed_vehicles.size(); ++i)
		{
			node_index = node_ID_to_index.at(stoi(passed_vehicles[i])); 
      one_function[node_index] = vehicle_beta[node_index];
		}
		one_function.back() = stod(ss[1]);
		coef.push_back (one_function);
	}
}

void DataManagementHelper::MessageHandleInstance (string s, int vehicle_id)
{
  vector<string> ss = split(s, "|");
  //NS_LOG_INFO("length of splitted vecotrs:"<<ss.size());
  if (ss.size() == num_entries+1) //original packet, I am the first router looks like IT|A|B|C|D
  {
    //NS_LOG_INFO("Original Packet");
    for (int i = 1; i < ss.size(); i++)
    {
      //NS_LOG_INFO("AddObserListInstance");
      obser_list_instance[vehicle_id].push_back(stod(ss[i]));
    }
  }else if (ss.size() == num_entries+2)//forwarded by others looks like IT|node1+node2+node3|A|B|C|D
  {
    NS_LOG_INFO("Forwared Packet");
    NS_LOG_INFO(s);
    vector<double> one_function(num_obser_expected+2,0);
    vector<string> passed_vehicles;
    for (int entries = 0; entries < num_entries; entries++) //handle each entries separately
    {
      //NS_LOG_INFO("passed_vehicles:"<<ss[1]);
      passed_vehicles = split(ss[1], "+");//node1+node2+node3
      for (int i=0; i<passed_vehicles.size(); ++i)
      {
        one_function[stoi(passed_vehicles[i])] = vehicle_beta[stoi(passed_vehicles[i])];
      }
      //NS_LOG_FUNCTION(entries<<ss[2+entries]);
      one_function.back() = stod(ss[2+entries]);
      //NS_LOG_FUNCTION(one_function);
      coef_instance[entries].push_back (one_function);
      //NS_LOG_INFO("Finished here. Entried:"<< entries <<"coef_instance[entries]: ");
      //NS_LOG_FUNCTION(coef_instance[entries]);

    }
  }


}

vector<vector<double> > DataManagementHelper::GetCoef()
{
	return coef;
}

vector<vector <vector<double>> > DataManagementHelper::GetCoefInstance()
{
  return coef_instance;
}

void DataManagementHelper::FunctionsClean ()
{
	int i,j;
    // rows
    for (i = 0; i < coef.size(); i++)
    {
        for(j = 0; j < coef[0].size()-1; j++)
        {
        	if ((coef[i][j] != 0) && (obser_list[j] != -100)) // we already known the observation value, so remove it
        	{
        		coef[i].back() = coef[i].back() - coef[i][j]*obser_list[j];
                if (abs(coef[i].back() - 0) < 0.0001) 
                    coef[i].back() = 0;
        		coef[i][j] = 0;
        	}
        }

    }


    vector<vector<double> > coef_1;

 	for (i = 0; i < coef.size(); i++)
    {
        if ( !all_of(coef[i].begin(), coef[i].end(), [](double i) { return abs(i-0.0)<= 0.000001; }))
        {
        	coef_1.push_back(coef[i]);
        }
    }

    if (coef_1.empty())
    {
        coef.assign(coef_1.begin(), coef_1.end());
        return;
    }



    vector <vector<double> > coef_2 (coef_1.size(),vector<double>(1, 0));

    for (j=0;j<coef_1[0].size()-1; j++)
    {
    	 for (int i=0; i<coef_1.size(); i++)
    	 {
    	 	
    	 	if ( abs(coef_1[i][j]-0.0) > 0.000001 )
    	 	{
    	 		for (int line=0; line<coef_1.size(); line++)
    	 		{
    	 			coef_2[line].insert(coef_2[line].end()-1, coef_1[line][j]);
    	 		}
                vehicle_id_can_calculate.push_back(j);
    	 		break;
    	 	}
    	 }
    }
    for (int line=0; line<coef_1.size(); line++)
    {
    	coef_2[line][coef_2[0].size()-1] = coef_1[line][coef_1[0].size()-1];//the last column
    }

    coef_2.erase(std::unique(coef_2.begin(), coef_2.end()), coef_2.end());//remove the duplicated functions

    if (coef_2.empty())
    {
        coef.assign(coef_2.begin(), coef_2.end());
        return;
    }


    coef.assign(coef_2.begin(), coef_2.end());

    //now to clean the columns

}



void DataManagementHelper::FunctionsCleanInstance ()
{
  //for each entries, clean the functions

  int i,j;
  for (int entries = 0; entries < num_entries; entries++)
  {
    NS_LOG_INFO("entries:"<<entries<<"Current_coef:");
    vector<vector<double>> current_coef = coef_instance[entries];
    NS_LOG_FUNCTION(current_coef);
    vector<vector<double> > coef_1;
    // rows
    for (i = 0; i < current_coef.size(); i++)
    {
        for(j = 0; j < current_coef[0].size()-1; j++)
        {
          //NS_LOG_FUNCTION(current_coef[i][j]<<obser_list_instance[j][entries]);
          if ((current_coef[i][j] != 0) && (obser_list_instance[j][entries] != -100)) // we already known the observation value, so remove it
          {
            current_coef[i].back() = current_coef[i].back() - current_coef[i][j]*obser_list_instance[j][entries];
            if (abs(current_coef[i].back() - 0) < 0.0001) //to handle the problem of "double" in cpp
              current_coef[i].back() = 0;
            current_coef[i][j] = 0;
            //NS_LOG_INFO("current_coef[i].back():"<<current_coef[i].back());
          }
        }
    }
    for (i = 0; i < current_coef.size(); i++)
    {
        if ( !all_of(current_coef[i].begin(), current_coef[i].end(), [](double i) { return abs(i-0.0)<= 0.000001; }))
        {
          //NS_LOG_INFO("Test here1");
          coef_1.push_back(current_coef[i]);
          //NS_LOG_INFO("Test here2");
        }
    }
    if (coef_1.empty())
    {
        //NS_LOG_INFO("Test here3");
        coef_instance[entries].assign(coef_1.begin(), coef_1.end());
        return;
    }
    /*
    NS_LOG_LOGIC("Print coef_1: ");//cout << "Print coef_1: " << endl;
    for (i = 0; i < coef_1.size(); i++)
    {
      for(j = 0; j < coef_1[0].size(); j++)
          NS_LOG_LOGIC(coef_1[i][j] );//cout << coef_1[i][j] << " ";
        //cout << endl;
    }*/
    vector <vector<double> > coef_2 (coef_1.size(),vector<double>(1, 0));
    NS_LOG_INFO("Test here4. coef_1:");
    NS_LOG_FUNCTION(coef_1);
    for (j=0;j<coef_1[0].size()-1; j++)
    {
       for (int i=0; i<coef_1.size(); i++)
       {
        
        if ( abs(coef_1[i][j]-0.0) > 0.000001 )
        {  
          for (int line=0; line<coef_1.size(); line++)
          {
            coef_2[line].insert(coef_2[line].end()-1, coef_1[line][j]);
          }
          vehicle_id_can_calculate_instance[entries].push_back(j);
          NS_LOG_INFO("Test here5.");
          break;
        }
       }
    }
    NS_LOG_INFO("Test here6.");
    for (int line=0; line<coef_1.size(); line++)
    {
        coef_2[line][coef_2[0].size()-1] = coef_1[line][coef_1[0].size()-1];//the last column
    }

    coef_2.erase(std::unique(coef_2.begin(), coef_2.end()), coef_2.end());//remove the duplicated functions

    NS_LOG_INFO("Test here7.");
    if (coef_2.empty())
    {
        coef_instance[entries].assign(coef_2.begin(), coef_2.end());
        return;
    }

   /* NS_LOG_LOGIC("Print coef_2: ");//cout << "Print coef_2: " << endl;
    for (i = 0; i < coef_2.size(); i++)
    {
      for(j = 0; j < coef_2[0].size(); j++)
          NS_LOG_LOGIC(coef_2[i][j]);//cout << coef_2[i][j] << " ";
        //cout << endl;
    }*/

    NS_LOG_INFO("Test here8.");
    coef_instance[entries].assign(coef_2.begin(), coef_2.end());
    NS_LOG_INFO("Test here9.");

  }

}


vector<int> DataManagementHelper::GetID_Map()
{
    return vehicle_id_can_calculate;
};

vector<vector<int>> DataManagementHelper::GetID_Map_Instance()
{
    return vehicle_id_can_calculate_instance;
};




//----------------------------------------------------------------------
//-- TimestampTag
//------------------------------------------------------


TypeId 
TimestampTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("TimestampTag")
    .SetParent<Tag> ()
    .AddConstructor<TimestampTag> ()
    .AddAttribute ("Timestamp",
                   "Some momentous point in time!",
                   EmptyAttributeValue (),
                   MakeTimeAccessor (&TimestampTag::GetTimestamp),
                   MakeTimeChecker ())
  ;
  return tid;
}
TypeId 
TimestampTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t 
TimestampTag::GetSerializedSize (void) const
{
  return 8;
}
void 
TimestampTag::Serialize (TagBuffer i) const
{
  int64_t t = m_timestamp.GetNanoSeconds ();
  i.Write ((const uint8_t *)&t, 8);
}
void 
TimestampTag::Deserialize (TagBuffer i)
{
  int64_t t;
  i.Read ((uint8_t *)&t, 8);
  m_timestamp = NanoSeconds (t);
}

void
TimestampTag::SetTimestamp (Time time)
{
  m_timestamp = time;
}
Time
TimestampTag::GetTimestamp (void) const
{
  return m_timestamp;
}

void 
TimestampTag::Print (std::ostream &os) const
{
  os << "t=" << m_timestamp;
}


