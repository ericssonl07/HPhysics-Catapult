#include <units.hpp>

Unit::Unit(std::array<int, NUM_BASE_DIMENSIONS> dims, double factor):
    m_dimensions_(dims), m_conversion_factor_(factor) {}

Unit::Unit(const Unit& other):
    m_dimensions_(other.m_dimensions_), m_conversion_factor_(other.m_conversion_factor_) {}

Unit::Unit(Unit&& other):
    m_dimensions_(std::move(other.m_dimensions_)), m_conversion_factor_(std::move(other.m_conversion_factor_)) {}

Unit& Unit::operator= (const Unit& other) {
    m_dimensions_ = other.m_dimensions_;
    m_conversion_factor_ = other.m_conversion_factor_;
    return *this;
}

Unit& Unit::operator= (Unit&& other) {
    m_dimensions_ = std::move(other.m_dimensions_);
    m_conversion_factor_ = std::move(other.m_conversion_factor_);
    return *this;
}

Unit::~Unit() = default;

const std::array<int, NUM_BASE_DIMENSIONS>& Unit::dimensions() const { return m_dimensions_; }

double Unit::conversion_factor() const { return m_conversion_factor_; }

bool Unit::is_compatible_with(const Unit& other) const {
    return m_dimensions_ == other.m_dimensions_;
}

void Unit::assert_is_compatible_with(const Unit& other, std::string error) const {
    if (!is_compatible_with(other)) {
        throw std::invalid_argument(error);
    }
}

Unit operator*(const Unit& a, const Unit& b) {
    std::array<int, NUM_BASE_DIMENSIONS> new_dims;
    for(size_t i = 0; i < NUM_BASE_DIMENSIONS; ++i) {
        new_dims[i] = a.m_dimensions_[i] + b.m_dimensions_[i];
    }
    return {new_dims, a.m_conversion_factor_ * b.m_conversion_factor_};
}

Unit operator/(const Unit& a, const Unit& b) {
    std::array<int, NUM_BASE_DIMENSIONS> new_dims;
    for(size_t i = 0; i < NUM_BASE_DIMENSIONS; ++i) {
        new_dims[i] = a.m_dimensions_[i] - b.m_dimensions_[i];
    }
    return {new_dims, a.m_conversion_factor_ / b.m_conversion_factor_};
}

Unit operator*(double factor, const Unit& u) {
    return {u.m_dimensions_, u.m_conversion_factor_ * factor};
}

Unit operator/(const Unit& u, double factor) {
    return {u.m_dimensions_, u.m_conversion_factor_ / factor};
}

bool Unit::operator==(const Unit& other) const {
    return m_dimensions_ == other.m_dimensions_;
}


double Quantity::base_value() const {
    return m_value_ * m_unit_.conversion_factor();
}

Quantity::Quantity(double value, const Unit& unit) : m_value_(value), m_unit_(unit) {}

Quantity::Quantity(const Quantity& other) : m_value_(other.m_value_), m_unit_(other.m_unit_) {}

Quantity::Quantity(Quantity&& other) : m_value_(std::move(other.m_value_)), m_unit_(std::move(other.m_unit_)) {}

Quantity& Quantity::operator=(const Quantity& other) {
    m_value_ = other.m_value_;
    m_unit_ = other.m_unit_;
    return *this;
}

Quantity& Quantity::operator=(Quantity&& other) {
    m_value_ = std::move(other.m_value_);
    m_unit_ = std::move(other.m_unit_);
    return *this;
}

Quantity::~Quantity() = default;

Quantity Quantity::convert_to(const Unit& new_unit) const {
    try {
        m_unit_.assert_is_compatible_with(new_unit, "Incompatible units for conversion");
    } catch (std::invalid_argument& e) {
        std::cout << e.what() << std::endl;
        for (int i = 0; i < NUM_BASE_DIMENSIONS; i++) {
            std::cout << m_unit_.dimensions()[i] << " ";
        }
        std::cout << std::endl;
        for (int i = 0; i < NUM_BASE_DIMENSIONS; i++) {
            std::cout << new_unit.dimensions()[i] << " ";
        }
    }
    return {base_value() / new_unit.conversion_factor(), new_unit};
}

Quantity Quantity::as(const Unit& new_unit) const {
    return convert_to(new_unit);
}

void Quantity::assert_is_compatible_with(const Unit& other, std::string error) const {
    m_unit_.assert_is_compatible_with(other, error);
}

void Quantity::assert_is_compatible_with(const Quantity& other, std::string error) const {
    assert_is_compatible_with(other.m_unit_, error);
}

Quantity Quantity::operator+(const Quantity& other) const {
    m_unit_.assert_is_compatible_with(other.m_unit_, "Incompatible units for addition");
    double sum_in_base = base_value() + other.base_value();
    return {sum_in_base / m_unit_.conversion_factor(), m_unit_};
}

Quantity Quantity::operator+=(const Quantity& other) {
    return *this = *this + other;
}

Quantity Quantity::operator-(const Quantity& other) const {
    m_unit_.assert_is_compatible_with(other.m_unit_, "Incompatible units for subtraction");
    double diff_in_base = base_value() - other.base_value();
    return {diff_in_base / m_unit_.conversion_factor(), m_unit_};
}

Quantity Quantity::operator-=(const Quantity& other) {
    return *this = *this - other;
}

Quantity Quantity::operator*(const Quantity& other) const {
    Unit new_unit = m_unit_ * other.m_unit_;
    return {m_value_ * other.m_value_, new_unit};
}

Quantity Quantity::operator*= (const Quantity& other) {
    return *this = *this * other;
}

Quantity Quantity::operator/(const Quantity& other) const {
    Unit new_unit = m_unit_ / other.m_unit_;
    return {m_value_ / other.m_value_, new_unit};
}

Quantity Quantity::operator/= (const Quantity& other) {
    return *this = *this / other;
}

Quantity Quantity::operator*(double factor) const {
    return {m_value_ * factor, m_unit_};
}

Quantity Quantity::operator*=(double factor) {
    return *this = *this * factor;
}

Quantity Quantity::operator/(double factor) const {
    return {m_value_ / factor, m_unit_};
}

Quantity Quantity::operator/=(double factor) {
    return *this = *this / factor;
}

Quantity operator*(double factor, const Quantity& q) {
    return {factor * q.m_value_, q.m_unit_};
}

Quantity operator/(double factor, Quantity& q) {
    return {factor / q.m_value_, Unit::dimensionless() / q.m_unit_};
}

bool Quantity::operator==(const Quantity& other) const {
    m_unit_.assert_is_compatible_with(other.m_unit_, "Incompatible units for comparison");
    return std::abs(base_value() - other.base_value()) < 1e-9;
}

std::ostream& operator<<(std::ostream& os, const Quantity& q) {
    return os << q.m_value_ << " [" << q.m_unit_.conversion_factor() << "]";
}

std::ostream& operator<<(std::ostream& os, Quantity&& q) {
    return os << q.m_value_ << " [" << q.m_unit_.conversion_factor() << "]";
}

const double& Quantity::value() const { return m_value_; }

const Unit& Quantity::unit() const { return m_unit_; }