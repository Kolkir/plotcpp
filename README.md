# plotcpp
This is a thin C++ wrapper for gnuplot program, it was made to simplify ploting data from C++ programms in binary format. Now it supprot only a small subset of gnuplot functionality, but allows to pass any gnuplot command as string. 

**Usage example**
```cpp  
  std::vector<float> y;
  std::vector<float> x;  

  Plot plt(persist_gnuplot_window);
  plt.SetTerminal("qt");
  plt.SetTitle("Sine");
  plt.SetXLabel("X");
  plt.SetYLabel("Y");
  plt.SetAutoscale();

  plt.Draw2D(Lines(x.begin(), x.end(), y.begin(), "lines"),
             Points(x.begin(), x.end(), y.begin(), "points"));

  plt.Flush();
```
![output](/example.png)

Please look at ```example.cpp``` file for details.
