# CUDA_Remeshing

논문 : https://www.dbpia.co.kr/journal/articleDetail?nodeId=NODE11213480
 - Triangle Mesh가 잘렸을 때 Remesh를 GPU로 가속화
 - 이 프로젝트를 진행하면서 변형구배와 메쉬구조에 대한 이해도가 높아짐

ClothDynamics는 Projective Dynamics기법을 Chebyshev Semi-Iterative기법으로 가속화 하여 구현

Continuous Material 기법을 통해 Constraint 구현

# Scene
#### 1
![scene1](https://user-images.githubusercontent.com/86860544/228156938-c8a0127d-5e78-4785-a2a5-c408e8a1b396.gif)

#### 2
![scene2](https://user-images.githubusercontent.com/86860544/228156956-75bf644d-7365-47e5-bace-926417c6fce7.gif)

# 참고문헌
 - H. Wang. A chebyshev semi-iterative approach for accelerating projective and position-based dynamics. ACM Trans. Graph., vol. 34, no. 6, pp. 246:1–246:9, Oct. 2015.
 - Jan Bendera, Dan Koschiera, Patrick Charriera and Daniel Weber. Position-Based Simulation of Continuous Materials. May 21, 2015.
