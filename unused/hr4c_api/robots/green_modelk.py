# -*- coding: utf-8 -*-
from hr4c_api.robots.green_6axis import Green6Axis


class GreenModelK(Green6Axis):
    """
    GreenModelKのフロントエンドクラス。
    """
    def __init__(self, server_ip):
        """
        初期化メソッド
        :param server_ip: コントローラBOX内制御計算機のIPアドレス
        """
        # 設計パラメータ
        h1 = 0.174 + 0.15
        l1 = 0.3
        l2 = 0.25
        l3 = 0.08
        min_angles = [-2.35, -1.0472, 0.392, -1.57, -1.57, -31.4]
        max_angles = [2.35, 2.26893, 2.74889, 1.57, 1.57, 31.4]
        m1 = 0
        m2 = 1.485  # linkA(0.635) + MDH6018(0.55) + control board(0.1) + other parts(0.2)
        m3 = 0.7
        m4 = 0
        m5 = 0.64
        m6 = 0

        # 親クラスの初期化
        super().__init__(server_ip, h1, l1, l2, l3, min_angles, max_angles, m1, m2, m3, m4, m5, m6)
