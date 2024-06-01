#ifndef RUI_EQUATION_CAL_H
#define RUI_EQUATION_CAL_H
#include<iostream>
#include<vector>
#include "ns3/stats-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
using namespace std;
using namespace ns3;
class DataRecoveryHelper 
{
public:
    DataRecoveryHelper (void);
    vector<double> GetResults();
    void SetParameters(int num_v, int num_f, vector<vector<double> > coe);//for decoding calculation
    void pc();//for calculation


private:
    
    void Allmin(int s, int n);
    void Allmake1(int n);
    void make1(int h, int n);
    void setloc(int h, int n);
    void Onereplace(int n);
    void min(int s,int h,int n);
    void Allreplace();
    void cacular(int n);
    void Allcacular();
   
    int N;//number of variables
    int M;//number of functions
    vector<double> s;//Result
    vector<vector<double> > m;//all coefficients
    vector<int> loc;
};

// Rui:Split， pattern is the split flag 
vector<string> split(const string &str, const string &pattern);

class DataManagementHelper
{
public:
    DataManagementHelper (int num_obser);
    vector<double> GetObserList();
    vector <vector<double>> GetObserListInstance();
    void AddObserList (double ob_value, int vehicle_id);
    void AddObserListInstance (double ob_value, int vehicle_id);
    void MessageHandle (string s, int vehicle_id);
    void MessageHandleInstance (string s, int vehicle_id);
    void FunctionsClean ();
    void FunctionsCleanInstance ();
    vector<vector<double> > GetCoef();
    vector<vector<vector<double>>> GetCoefInstance();
    vector<int> GetID_Map();
    vector<vector<int>> GetID_Map_Instance();
private:
    int num_obser_expected;
    vector<double> obser_list;
    vector<vector<double>> obser_list_instance;
    vector<vector<double> > coef;//all coefficients
    vector< vector<vector<double> > > coef_instance;
    vector<int> vehicle_id_can_calculate;
    vector<vector<int>> vehicle_id_can_calculate_instance;

};

class TimestampTag : public Tag {
public:
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;

  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (TagBuffer i) const;
  virtual void Deserialize (TagBuffer i);

  // these are our accessors to our tag structure
  void SetTimestamp (Time time);
  Time GetTimestamp (void) const;

  void Print (std::ostream &os) const;

private:
  Time m_timestamp;

  // end class TimestampTag
};

#endif 