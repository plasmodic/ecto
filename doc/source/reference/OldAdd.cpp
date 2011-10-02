struct Add
{
  static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    inputs.declare<double> ("left", "left input");
    inputs.declare<double> ("right", "right input");
    outputs.declare<double> ("out", "output");
  }

  void configure(const tendrils& p, const tendrils& i, const tendrils& o)
  {
    out_ = o["out"];
    left_ = i["left"];
    right_ = i["right"];
  }

  int process(const tendrils& /*inputs*/, const tendrils& /*outputs*/)
  {
    *out_ = (*left_ + *right_);
    return ecto::OK;
  }
  ecto::spore<double> out_, left_, right_;
};
