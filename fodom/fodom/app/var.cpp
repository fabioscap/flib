#include <iostream>

#include "fodom/common.h"
#include "fodom/utils.h"

using namespace fodom;
ChronoStopWatch sw;

size_t base_counter = 0;

template <typename T_>
struct Base_ {
  using T = T_;
  T data;

  Base_() {
    base_counter++;
    std::cout << "Base_()\n";
  }
  Base_(const T& d) : data(d) {
    base_counter++;
    std::cout << "Base_(const T& d)\n";
  }
  Base_(T&& d) noexcept : data(std::move(d)) {
    base_counter++;
    std::cout << "Base_(T&& d)\n";
  }
  Base_(const Base_& other) : data(other.data) {
    base_counter++;
    std::cout << "Base_(const Base_& other)\n";
  }
  Base_(Base_&& other) noexcept : data(std::move(other.data)) {
    base_counter++;
    std::cout << "Base_(Base_&& other)\n";
  }
};

template <typename T_>
struct Base2_ {
  using T = T_;
  T data1;
  T data2;

  Base2_() {
    base_counter++;
    std::cout << "Base2_()\n";
  }
  Base2_(const T& d) : data1(d), data2(-d) {
    base_counter++;
    std::cout << "Base2_(const T& d)\n";
  }
  Base2_(const T& d, const T& d2) : data1(d), data2(d2) {
    base_counter++;
    std::cout << "Base2_(const T& d, const T&d2)\n";
  }
  Base2_(T&& d) noexcept : data1(std::move(d)), data2(std::move(-d)) {
    base_counter++;
    std::cout << "Base2_(T&& d)\n";
  }
  Base2_(const Base2_& other) : data1(other.data1), data2(other.data2) {
    base_counter++;
    std::cout << "Base2_(const Base2_& other)\n";
  }
  Base2_(Base2_&& other) noexcept
      : data1(std::move(other.data1)), data2(std::move(other.data2)) {
    base_counter++;
    std::cout << "Base2_(Base2_&& other)\n";
  }
};

template <typename... Members_>
struct Container_;

template <typename Member_>
struct Container_<Member_> {
  using MemberType = Member_;

  MemberType m;

  Container_() { std::cout << "Container_()\n"; }

  template <typename Arg>
  Container_(Arg&& arg) : m(std::forward<Arg>(arg)) {
    std::cout << "Container_(Arg&& arg)\n";
  }
};

template <typename Member_, typename... Rest_>
struct Container_<Member_, Rest_...> {
  using MemberType = Member_;
  using RestType = Container_<Rest_...>;

  MemberType m;
  RestType r;

  Container_() { std::cout << "ContainerMR_()\n"; }

  template <typename FirstArg, typename... RestArgs>
  Container_(FirstArg&& firstArg, RestArgs&&... restArgs)
      : m(std::forward<FirstArg>(firstArg)),
        r(std::forward<RestArgs>(restArgs)...) {
    std::cout << "ContainerMR_(FirstArg&& firstArg, RestArgs&&... restArgs)\n";
  }
};

template <typename... Members_>
struct Container2_;

template <typename Member_>
struct Container2_<Member_> {
  using MemberType = Member_;

  MemberType m;

  Container2_() { std::cout << "Container_()\n"; }

  template <typename Arg>
  Container2_(Arg&& arg) : m(std::forward<Arg>(arg)) {
    std::cout << "Container_(Arg&& arg)\n";
  }

  Container2_ f() const { return Container2_(1.0); }
};

template <typename Member_, typename... Rest_>
struct Container2_<Member_, Rest_...> : Container2_<Rest_...> {
  using MemberType = Member_;
  using BaseType = Container2_<Rest_...>;
  MemberType m;

  Container2_() { std::cout << "Container2MR_()\n"; }

  template <typename FirstArg, typename... RestArgs>
  Container2_(FirstArg&& firstArg, RestArgs&&... restArgs)
      : BaseType(std::forward<RestArgs>(restArgs)...),
        m(std::forward<FirstArg>(firstArg)) {
    std::cout << "Container2MR_(FirstArg&& firstArg, RestArgs&&... restArgs)\n";
  }

  Container2_ f() const {
    return Container2_(1.0, BaseType::f().m);  // what to put here?)
  }
};

int main() {
  using Base = Base_<double>;
  using Base2 = Base2_<double>;
  using Container = Container2_<Base, Base, Base2>;

  Container c2{1.1, 2.2, 3.3};  // Using constructor for forwarding arguments
  std::cout << "nr of Base_ created: " << base_counter << std::endl;

  c2.f();
  std::cout << "nr of Base_ created: " << base_counter << std::endl;

  return 0;
}