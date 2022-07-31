from typing import List

from ..quadricslam_states import Detection, QuadricSlamState
from . import Detector

try:
    import detectron2 as d2
except:
    raise ImportError("Failed to import 'detectron2'. Please install from:"
                      "\n\thttps://github.com/facebookresearch/detectron2")
from detectron2 import config as d2c
from detectron2 import data as d2d
from detectron2 import engine as d2e
from detectron2 import model_zoo as d2mz
from detectron2.utils import visualizer as d2v


class FasterRcnn(Detector):

    def __init__(
            self,
            zoo_model: str = 'COCO-Detection/faster_rcnn_R_50_FPN_1x.yaml',
            detection_thresh: float = 0.5) -> None:
        c = d2c.get_cfg()
        c.merge_from_file(d2mz.get_config_file(zoo_model))
        c.MODEL.ROI_HEADS.SCORE_THRESH_TEST = detection_thresh
        c.MODEL.WEIGHTS = d2mz.get_checkpoint_url(zoo_model)
        self.predictor = d2e.DefaultPredictor(c)
        self.classes = d2d.MetadataCatalog.get(
            self.predictor.cfg.DATASETS.TRAIN[0]).thing_classes

    def detect(self, state: QuadricSlamState) -> List[Detection]:
        assert state.this_step is not None
        assert state.this_step.rgb is not None
        n = state.this_step

        inst = self.predictor(n.rgb)['instances']
        pred_classes = inst.get('pred_classes').detach().cpu().numpy()
        pred_boxes = inst.get('pred_boxes').tensor.detach().cpu().numpy()
        return [
            Detection(label=self.classes[pred_classes[i]],
                      bounds=pred_boxes[i],
                      pose_key=n.pose_key)
            for i in range(0, len(pred_classes))
        ]
