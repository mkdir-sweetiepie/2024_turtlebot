#include "../include/robit_vision/robit_ransac.hpp"

#include <chrono>
using namespace std;

// JYJ_RansacLine 클래스 생성자
// 성공률, 내부점 비율, 거리 임계값, 길이 임계값을 매개변수로 받음
JYJ_RansacLine::JYJ_RansacLine(const double &success_rate, const double &inlier_rate, const double &distance_threshold, const double &length_threshold) {
  // 기존 이미지가 있다면 메모리 해제
  if (!m_Image.empty()) m_Image.release();

  // 초기 변수 설정
  m_done = false;
  m_color = false;

  m_height = 0;
  m_width = 0;
  m_inlier_rate = inlier_rate;
  m_success_rate = success_rate;
  m_length_threshold = length_threshold;
  m_distance_threshold = distance_threshold;
}

// JYJ_RansacLine 클래스의 두 번째 생성자
// 이미지와 여러 매개변수를 받아 초기화
JYJ_RansacLine::JYJ_RansacLine(const cv::Mat &Img, const bool bAutoThreshold, const bool is_right, const double &success_rate, const double &inlier_rate, const double &distance_threshold,
                               const double &length_threshold, const double length_threshold_max) {
  // 이미지 데이터 포인터 저장
  m_img_data = (unsigned char *)Img.data;
  m_done = false;
  m_color = false;

  // 이미지 크기 저장
  m_height = Img.rows;
  m_width = Img.cols;

  // 기본 임계값 설정
  m_cost_threshold = 0.4;
  m_length_threshold_max = length_threshold_max;

  // 자동 임계값 설정 여부에 따라 변수 초기화
  if (bAutoThreshold == true) {
    // 자동 임계값 설정
    m_success_rate = 0.99;
    m_length_threshold = Img.rows / 4.0;
    m_distance_threshold = 1.5;
    m_cost_threshold = 0.4;
    m_is_right = is_right;
    m_bAutoThreshold = true;
  } else {
    // 수동 임계값 설정
    m_inlier_rate = inlier_rate;
    m_success_rate = success_rate;
    m_length_threshold = length_threshold;
    m_distance_threshold = distance_threshold;
    m_bAutoThreshold = false;
  }
}

// RANSAC 알고리즘 실행 함수
void JYJ_RansacLine::runRansac() {
  // 이미지가 그레이스케일인지 확인
  if (m_Image.channels() != 1) {
    std::cout << "Image type is not collect" << std::endl;
    return;
  }
  // 엣지 포인트 추출
  _getEdgePoint();

  // 엣지 포인트 개수에 따른 내부점 비율 계산
  unsigned int edge_point_size = m_edge_point.size();
  m_inlier_rate = ((double)m_height / 3.0) / (double)edge_point_size;

  if (edge_point_size < 2) return;
  srand(time(NULL));

  double max_cost = 0.0;
  // 최대 반복 횟수 계산
  unsigned int max_iteration = _getMaxIteration(edge_point_size);

  // 최대 반복 횟수 제한
  if (max_iteration > 1500) max_iteration = 1500;

  m_done = false;
  // RANSAC 메인 루프
  for (int i = 0; i < max_iteration; i++) {
    vector<cv::Point> sample_point;

    // 샘플 포인트 선택
    if (_getSamples(sample_point, edge_point_size) == false) continue;

    // 라인 비용 계산
    double cost = _getLineCost(sample_point, edge_point_size);

    // 최적의 라인 갱신
    if (cost > m_cost_threshold) {
      if (max_cost < cost) {
        max_cost = cost;
        m_line_start = sample_point[0];
        m_line_end = sample_point[1];
        m_done = true;
      }
    }
  }

  // 라인의 시작점과 끝점 조정
  if (m_line_start.x < m_line_end.x) {
    cv::Point temp_point = m_line_start;
    m_line_start = m_line_end;
    m_line_end = temp_point;
  }

  // 라인의 각도 계산
  m_degree = -(atan2(m_line_start.y - m_line_end.y, m_line_start.x - m_line_end.x) * 180.0) / CV_PI;
  if (m_degree < 0.0) m_degree += 180.0;

  // y 좌표 기준으로 시작점과 끝점 조정
  if (m_line_start.y < m_line_end.y) {
    cv::Point temp_point = m_line_start;
    m_line_start = m_line_end;
    m_line_end = temp_point;
  }
}

// 이미지에서 엣지 포인트 추출 함수
void JYJ_RansacLine::_getEdgePoint() {
  for (int nRow = 0, nowY = 0; nRow < m_height; nRow++, nowY += m_width)
    for (int nCol = 0; nCol < m_width; nCol++) {
      if (*(m_img_data + nowY + nCol) != 0) m_edge_point.push_back(cv::Point(nCol, nRow));
    }
}

// 샘플 포인트 선택 함수
bool JYJ_RansacLine::_getSamples(vector<cv::Point> &sample_point, const unsigned int &edge_point_size) {
  // 랜덤하게 두 점 선택
  const int n = rand() % edge_point_size;
  const int m = rand() % edge_point_size;

  // 유효성 검사
  if (n == m) return false;
  if (m_bAutoThreshold && m_edge_point[n].y < m_height * 0.9 && m_edge_point[m].y < m_height * 0.9) return false;
  if (norm(m_edge_point[n] - m_edge_point[m]) < m_length_threshold || norm(m_edge_point[n] - m_edge_point[m]) > m_length_threshold_max) return false;

  sample_point.push_back(m_edge_point[n]);
  sample_point.push_back(m_edge_point[m]);

  // 라인 각도 계산
  double temp_degree = 0.0;

  if (m_edge_point[n].x < m_edge_point[m].x) {
    cv::Point temp_point = m_edge_point[n];
    m_edge_point[n] = m_edge_point[m];
    m_edge_point[m] = temp_point;
  }
  temp_degree = -(atan2(m_edge_point[n].y - m_edge_point[m].y, m_edge_point[n].x - m_edge_point[m].x) * 180.0) / CV_PI;
  if (temp_degree < 0.0) temp_degree += 180.0;

  // 자동 임계값 설정 시 추가 검증
  if (m_bAutoThreshold == true) {
    if (!m_color) {
      if (m_is_right) {
        if (m_edge_point[n].x < m_width / 3 || m_edge_point[m].x < m_width / 3) return false;
        if (temp_degree < 85.0 || temp_degree > 165.0) return false;
      } else {
        if (m_edge_point[n].x > m_width * 2 / 3 || m_edge_point[m].x > m_width * 3 / 2) return false;
        if (temp_degree < 15.0 || temp_degree > 95.0) return false;
      }
    }
  } else {
    if (temp_degree > 5.0 && temp_degree < 175.0) {
      return false;
    }
  }

  return true;
}

// 라인 비용 계산 함수
const double JYJ_RansacLine::_getLineCost(vector<cv::Point> &sample_point, const unsigned int &edge_point_size) {
  // 라인 방정식 계수 계산
  double a = sample_point[0].y - sample_point[1].y;
  double b = sample_point[1].x - sample_point[0].x;
  double c = -(b * sample_point[0].y + a * sample_point[0].x);

  double sqrtLineEq = sqrt(pow(a, 2.0) + pow(b, 2.0));

  // 비용 계산
  double cost = 0;
  for (int i = 0; i < edge_point_size; i++) {
    const double distance = fabs((m_edge_point[i].x * a) + (m_edge_point[i].y * b) + c) / sqrtLineEq;

    if (distance < m_distance_threshold) cost++;
  }
  cost /= norm(sample_point[0] - sample_point[1]);
  return cost;
}

// 최대 반복 횟수 계산 함수
unsigned int JYJ_RansacLine::_getMaxIteration(const unsigned int &edge_point_size) { return log10(1.0 - m_success_rate) / log10(1.0 - pow(m_inlier_rate, 2)); }

// 소멸자
JYJ_RansacLine::~JYJ_RansacLine() { m_Image.release(); }