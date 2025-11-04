# course-slam-stachniss

Learning resources and code for the **SLAM (Simultaneous Localization and Mapping)** course by **Cyrill Stachniss**.

---

## 📚 About
This repository contains exercises, implementations, and notes based on the SLAM course taught by Cyrill Stachniss. The course covers fundamental concepts in robotics mapping and localization, including:

- Probabilistic robotics
- Bayes Filter
- EKF and UKF based SLAM
- Particle filters
- Occupancy grid mapping
- Graph-based SLAM

---

## 🔗 Official Resources
- [Course page](http://ais.informatik.uni-freiburg.de/teaching/ws13/mapping)
- [Youtube playlist](https://youtu.be/U6vr3iNrwRA?si=jVpTmYmCFViDadl9)

---

## 🚀 Getting Started
1. Install the requirements:
  ```bash
  sudo apt install octave
  ```

2. Clone the repository:
  ```bash
  git clone https://github.com/your-username/course-slam-stachniss.git
  cd course-slam-stachniss
  ```
---

## :wrench: Tools
- Generating mp4 and gifs:
  ```bash
  ffmpeg -framerate 10 -pattern_type glob -i '*.png' -c:v libx264 -pix_fmt yuv420p output.mp4
  ffmpeg -framerate 10 -pattern_type glob -i '*.png' output.gif
  ```
