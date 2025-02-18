#pragma once
#ifndef UNITS_HPP
#define UNITS_HPP

#include <array>
#include <cmath>
#include <stdexcept>
#include <iostream>

enum BaseDimension {
    LENGTH,
    MASS,
    TIME,
    ELECTRIC_CURRENT,
    TEMPERATURE,
    SUBSTANCE,
    LUMINOUS_INTENSITY,
    NUM_BASE_DIMENSIONS
};

#define M_PI 3.14159265358979323846

class Unit {
private:
    std::array<int, NUM_BASE_DIMENSIONS> m_dimensions_;
    double m_conversion_factor_;
public:
    Unit(std::array<int, NUM_BASE_DIMENSIONS> dims, double factor);
    Unit(const Unit& other);
    Unit(Unit&& other);
    Unit& operator= (const Unit& other);
    Unit& operator= (Unit&& other);
    ~Unit();

    // Dimensionless
    static Unit dimensionless()                 { return {{0,0,0,0,0,0,0}, 1.0}; }

    // SI base units
    static Unit meter()                         { return {{1,0,0,0,0,0,0}, 1.0}; }
    static Unit kilogram()                      { return {{0,1,0,0,0,0,0}, 1.0}; }
    static Unit second()                        { return {{0,0,1,0,0,0,0}, 1.0}; }
    static Unit ampere()                        { return {{0,0,0,1,0,0,0}, 1.0}; }
    static Unit kelvin()                        { return {{0,0,0,0,1,0,0}, 1.0}; }
    static Unit mole()                          { return {{0,0,0,0,0,1,0}, 1.0}; }
    static Unit candela()                       { return {{0,0,0,0,0,0,1}, 1.0}; }

    // Prefixes
    static double yocto()                       { return 1e-24; }
    static double zepto()                       { return 1e-21; }
    static double atto()                        { return 1e-18; }
    static double femto()                       { return 1e-15; }
    static double pico()                        { return 1e-12; }
    static double nano()                        { return 1e-9;  }
    static double micro()                       { return 1e-6;  }
    static double milli()                       { return 1e-3;  }
    static double centi()                       { return 1e-2;  }
    static double deci()                        { return 1e-1;  }
    static double deca()                        { return 1e1;   }
    static double hecto()                       { return 1e2;   }
    static double kilo()                        { return 1e3;   }
    static double mega()                        { return 1e6;   }
    static double giga()                        { return 1e9;   }
    static double tera()                        { return 1e12;  }
    static double peta()                        { return 1e15;  }
    static double exa()                         { return 1e18;  }
    static double zetta()                       { return 1e21;  }
    static double yotta()                       { return 1e24;  }

    // Area and volume
    static Unit square_meter()                  { return meter() * meter();                             }
    static Unit cubic_meter()                   { return meter() * meter() * meter();                   }
    // Linear motion
    static Unit meters_per_second()             { return meter() / second();                            }
    static Unit kilometers_per_hour()           { return (1000 * meter()) / (3600 * second());          }
    static Unit miles_per_hour()                { return 0.44704 * meters_per_second();                 }
    static Unit meters_per_second_squared()     { return meter() / (second() * second());               }
    // Force and pressure
    static Unit newton()                        { return kilogram() * meter() / (second() * second());  }
    static Unit pascal()                        { return newton() / (meter() * meter());                }
    static Unit bar()                           { return 100000 * pascal();                             }
    static Unit atmosphere()                    { return 101325 * pascal();                             }
    static Unit psi()                           { return 6894.76 * pascal();                            }
    // Energy and power
    static Unit joule()                         { return newton() * meter();                            }
    static Unit watt()                          { return joule() / second();                            }
    static Unit calorie()                       { return 4.184 * joule();                               }
    static Unit watt_hour()                     { return 3.6e3 * joule();                               }
    static Unit kilowatt_hour()                 { return 3.6e6 * joule();                               }
    static Unit horsepower()                    { return 745.7 * watt();                                }
    // Torque
    static Unit newton_meter()                  { return newton() * meter();                            }
    // Electrical units
    static Unit coulomb()                       { return ampere() * second();                           }
    static Unit volt()                          { return joule() / coulomb();                           }
    static Unit ohm()                           { return volt() / ampere();                             }
    static Unit farad()                         { return coulomb() / volt();                            }
    // Frequency
    static Unit hertz()                         { return dimensionless() / second();                    }
    // Angles
    static Unit radian()                        { return dimensionless();                               }
    static Unit degree()                        { return (M_PI / 180.0) * radian();                     }
    static Unit revolution()                    { return 2 * M_PI * radian();                           }
    static Unit rpm()                           { return revolution() / (60 * second());                }
    static Unit steradian()                     { return dimensionless();                               }
    
    const std::array<int, NUM_BASE_DIMENSIONS>& dimensions() const;
    double conversion_factor() const;
    bool is_compatible_with(const Unit& other) const;
    void assert_is_compatible_with(const Unit& other, std::string error = "Incompatible units") const;
    friend Unit operator*(const Unit& a, const Unit& b);
    friend Unit operator/(const Unit& a, const Unit& b);
    friend Unit operator*(double factor, const Unit& u);
    friend Unit operator/(const Unit& u, double factor);
    bool operator==(const Unit& other) const;
};

class Quantity {
private:
    double m_value_;
    Unit m_unit_;
    double base_value() const;
public:
    Quantity(double value, const Unit& unit);
    Quantity(const Quantity& other);
    Quantity(Quantity&& other);
    Quantity& operator=(const Quantity& other);
    Quantity& operator=(Quantity&& other);
    ~Quantity();
    Quantity convert_to(const Unit& new_unit) const;
    Quantity as(const Unit& new_unit) const;
    void assert_is_compatible_with(const Unit& other, std::string error = "Incompatible units") const;
    void assert_is_compatible_with(const Quantity& other, std::string error = "Incompatible units") const;
    Quantity operator+(const Quantity& other) const;
    Quantity operator+=(const Quantity& other);
    Quantity operator-(const Quantity& other) const;
    Quantity operator-=(const Quantity& other);
    Quantity operator*(const Quantity& other) const;
    Quantity operator*= (const Quantity& other);
    Quantity operator/(const Quantity& other) const;
    Quantity operator/= (const Quantity& other);
    Quantity operator*(double factor) const;
    Quantity operator*=(double factor);
    Quantity operator/(double factor) const;
    Quantity operator/=(double factor);
    friend Quantity operator*(double factor, const Quantity& q);
    friend Quantity operator/(double factor, Quantity& q);
    bool operator==(const Quantity& other) const;
    friend std::ostream& operator<<(std::ostream& os, const Quantity& q);
    friend std::ostream& operator<<(std::ostream& os, Quantity&& q);
    const double& value() const;
    const Unit& unit() const;
};

#endif // #ifndef UNITS_HPP