#ifndef LIDAR_ALIGN_TABLE_H_
#define LIDAR_ALIGN_TABLE_H_

#include <ncurses.h>

namespace lidar_align {

class Table {
 public:
  Table(const std::vector<std::string>& col_names, size_t names_width,
        size_t cell_width)
      : names_width_(names_width), cell_width_(cell_width) {
    initscr();
    curs_set(0);
    header_window_ = newwin(1, 80, 0, 0);
    wattron(header_window_, A_REVERSE);
    footer_window_ = newwin(3, 80, 0, 0);
    wattron(footer_window_, A_REVERSE);

    for (size_t col = 0; col < col_names.size(); ++col) {
      col_name_windows_.emplace_back(
          newwin(1, cell_width_, 1, names_width_ + col * cell_width_));
      wattron(col_name_windows_[col], A_BOLD);
      wprintw(col_name_windows_[col], col_names[col].c_str());
      wrefresh(col_name_windows_[col]);
    }
  }

  template <typename T>
  void updateHeader(const T& data) {
    std::stringstream ss;
    ss << std::left << std::setw(80) << std::setprecision(3) << data;
    wclrtoeol(header_window_);
    mvwprintw(header_window_, 0, 0, ss.str().c_str());
    wrefresh(header_window_);
  }

  template <typename T>
  void updateFooter(const T& data) {
    std::stringstream ss;
    ss << std::left << std::setw(80) << std::setprecision(3) << data;
    wclrtoeol(footer_window_);
    mvwprintw(footer_window_, 0, 0, ss.str().c_str());
    wrefresh(footer_window_);
  }

  template <typename T>
  void updateCell(const std::string row_name, const size_t col, const T& data) {
    setupRow(row_name);
    std::stringstream ss;
    ss << std::setprecision(3) << data;
    wclrtoeol(cell_windows_[row_name][col]);
    mvwprintw(cell_windows_[row_name][col], 0, 0, ss.str().c_str());
    wrefresh(cell_windows_[row_name][col]);
  }

  template <typename T>
  void updateRow(const std::string row_name, const std::vector<T>& data) {
    setupRow(row_name);

    for (size_t col = 0; col < col_name_windows_.size(); ++col) {
      std::stringstream ss;
      ss << std::setprecision(3) << data[col];
      wclrtoeol(cell_windows_[row_name][col]);
      mvwprintw(cell_windows_[row_name][col], 0, 0, ss.str().c_str());
      wrefresh(cell_windows_[row_name][col]);
    }
  }

 private:
  void setupRow(const std::string row_name) {
    if (cell_windows_.count(row_name) == 0) {
      mvwin(footer_window_, 3 + cell_windows_.size(), 0);
      wrefresh(footer_window_);

      size_t num_rows = cell_windows_.size();

      row_name_windows_.emplace_back(newwin(1, names_width_, 2 + num_rows, 0));
      wattron(row_name_windows_.back(), A_BOLD);
      wprintw(row_name_windows_.back(), row_name.c_str());
      wrefresh(row_name_windows_.back());
      for (size_t col = 0; col < col_name_windows_.size(); ++col) {
        cell_windows_[row_name].emplace_back(newwin(
            1, cell_width_, 2 + num_rows, names_width_ + cell_width_ * col));
      }
    }
  }

  WINDOW* header_window_;
  WINDOW* footer_window_;
  std::vector<WINDOW*> col_name_windows_;
  std::vector<WINDOW*> row_name_windows_;
  std::map<std::string, std::vector<WINDOW*>> cell_windows_;

  size_t names_width_;
  size_t cell_width_;
};

}  // namespace lidar_align

#endif
