struct MyEctoModule
{



struct stringv
{
  bool operator()(std::string value)
  {
    //....
    return true;
  }
};

struct dependent_v
{
  bool operator()(const tendrils& params, float value)
  {
    return params.get<float>("min") <= value <= params.get<float>("max");
  }
};

static void declare_params(tendrils& params)
{
  params.declare<std::string>("not_validated");
  params.declare<std::string>("validated").register_validator(stringv());
  params.declare<float>("max","maximum value, will clamp value", 10.0f);
  params.declare<float>("min","min value, will clamp value", 0.0f);
  params.declare<float>("value", "my awesome value", 5.0f).register_validator(dependent_v());
}

void str_cb(std::string s)
{
  s_ = s;
}

void str_cb_validated(std::string s)
{
  //stringv is called before this function is called... so that s is
  //definitely valid
  s_validated_ = s;
}

void configure(tendrils& params)
{
  params.register_callback(boost::bind(reconfigure, this,_1)); //global parameter change callback
  value_ = params.get<float>("value");
  params.at<std::string>("not_validated").register_callback(boost::bind(str_cb, this, _1));
  params.at<std::string>("validated").register_callback(boost::bind(str_cb_validated, this, _1));
}

void reconfigure(tendrils& params)
{
  if(params.get<float>("max") >= params.get<float>("value") )
    value_ = params.get<float>("value");
}

int process(const tendrils& inputs, tendrils& outputs)
{
  if (value_ > 1010)
  {
    //.... do stuff.
  }
  return 0;
}

float value_;
std::string s_,s_validated_;
};
