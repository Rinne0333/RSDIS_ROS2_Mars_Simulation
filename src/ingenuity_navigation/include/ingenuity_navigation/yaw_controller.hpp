#ifndef INGENUITY_NAVIGATION__YAW_CONTROLLER_HPP_
#define INGENUITY_NAVIGATION__YAW_CONTROLLER_HPP_

/**
 * @brief PD de guinada (yaw) com saída em NEWTONS diferenciais.
 *
 * NOTA FÍSICA: esta saída não é um binário. É uma diferença de empuxo
 * a aplicar nos rotores coaxiais: Top=base+diff, Bottom=base-diff.
 * O plugin Thruster do modelo pode NÃO produzir binário de yaw controlável.
 * Se não produzir, a fase ROTATE vai terminar por timeout, sem a saltar.
 */
class YawController
{
public:
    struct Config
    {
        double kp = 1.4;           // N/rad
        double kd = 0.5;           // N/(rad/s)
        double max_diff_n = 1.5;   // limite absoluto por rotor
    };

    explicit YawController(const Config &config);
    double update(double target_heading_world, double current_heading_world,
                  double yaw_rate_world) const;
    static double wrapAngle(double angle);

private:
    Config config_;
};

#endif
