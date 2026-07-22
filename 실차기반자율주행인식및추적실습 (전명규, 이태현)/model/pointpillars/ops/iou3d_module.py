import torch
import torchvision

def boxes_overlap_bev(boxes_a, boxes_b):
    iou = boxes_iou_bev(boxes_a, boxes_b)
    return iou

def boxes_iou_bev(boxes_a, boxes_b):
    beva = torch.zeros((boxes_a.shape[0], 4))
    beva[:, 0] = boxes_a[:, 0] - boxes_a[:, 2] / 2
    beva[:, 1] = boxes_a[:, 1] - boxes_a[:, 3] / 2
    beva[:, 2] = boxes_a[:, 0] + boxes_a[:, 2] / 2
    beva[:, 3] = boxes_a[:, 1] + boxes_a[:, 3] / 2

    bevb = torch.zeros((boxes_b.shape[0], 4))
    bevb[:, 0] = boxes_b[:, 0] - boxes_b[:, 2] / 2
    bevb[:, 1] = boxes_b[:, 1] - boxes_b[:, 3] / 2
    bevb[:, 2] = boxes_b[:, 0] + boxes_b[:, 2] / 2
    bevb[:, 3] = boxes_b[:, 1] + boxes_b[:, 3] / 2

    return torchvision.ops.box_iou(beva, bevb)

def nms_cuda(boxes, scores, thresh, pre_maxsize=None, post_max_size=None):
    order = scores.sort(0, descending=True)[1]
    if pre_maxsize is not None:
        order = order[:pre_maxsize]

    boxes = boxes[order]
    scores = scores[order]

    keep_boxes = torch.zeros((boxes.shape[0], 4))
    keep_boxes[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
    keep_boxes[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
    keep_boxes[:, 2] = boxes[:, 0] + boxes[:, 2] / 2
    keep_boxes[:, 3] = boxes[:, 1] + boxes[:, 3] / 2

    keep = torchvision.ops.nms(keep_boxes, scores, thresh)

    keep = order[keep]

    if post_max_size is not None:
        keep = keep[:post_max_size]

    return keep

def nms_normal_gpu(boxes, scores, thresh):
    return nms_cuda(boxes, scores, thresh)
