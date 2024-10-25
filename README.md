# ClothCutting
天池2019广东工业智造创新大赛，布料切割方案算法设计



## 环境依赖

- CMake (版本 3.10 或更高)
- GCC/G++ (支持 C++14)
- Boost 库 (包括 Geometry)
- Clipper 库 (已包含在项目中)
- libnfporb (已包含在项目中)



## TODO

- 检查算例中零件方向，统一为按照逆时针方向旋转；
- 使用 clipper 放大、简化多边形的顺序和必要性？
- libnfporb 实现有问题？
- 添加瑕疵，将瑕疵转换为多边形，同时也要参加差集运算；
- 改写为 cmake 工程，适配 Linux 平台，最终打包成 docker 镜像。



## 参考

Nest4J：https://github.com/Yisaer/Nest4J

libnfporb 参考论文：[E.K. Burke *et al.* 2006](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.440.379&rep=rep1&type=pdf)

基于可行解（贪心、模拟退火）：[Solving Irregular Strip Packing problems by hybridising simulated annealing and linear programming](https://www.sciencedirect.com/science/article/pii/S0377221704005879?via%3Dihub)

基于不可行解（重叠消除）：[An iterated local search algorithm based on nonlinear programming for the irregular strip packing problem](https://www.sciencedirect.com/science/article/pii/S1572528609000218?via%3Dihub)

Ps：目前实现的是基于可行解的贪心放置策略，几何运算开销较大，理论上基于不可行解的重叠消除效果可能更好。



## 编译步骤

1. 安装依赖:
   ```
   sudo apt-get update
   sudo apt-get install cmake g++ libboost-all-dev
   ```

2. 编译项目:
   ```
   mkdir build
   cd build
   cmake ..
   make
   ```

3. 运行程序:
   ```
   ./ClothCutting
   ```
