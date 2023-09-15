import { disableBodyScroll, enableBodyScroll } from 'body-scroll-lock';

export class ResizeHandle extends HTMLElement {
    static css = `
    resize-handle { position: absolute; }
    resize-handle:after { content: ' '; display: block; width: 12px; height: 12px; opacity: .5; }
    resize-handle:hover:after { width: 16px; height: 16px; opacity: 1; }
    resize-handle[top][left] { top: 1px; left: 1px; cursor: nw-resize; }
    resize-handle[top][left]:after {
      background: linear-gradient( -45deg, #0000, #0000 60%, #000 60%, #000, #0000, #0000 70%, #000 70%, #000, #0000, #0000 80%, #000 80%, #000, #0000, #0000 90%, #000 90%, #000 )
    }
    resize-handle[top][right] { top: 1px; right: 1px; cursor: ne-resize; }
    resize-handle[top][right]:after {
      background: linear-gradient( 45deg, #0000, #0000 60%, #000 60%, #000, #0000, #0000 70%, #000 70%, #000, #0000, #0000 80%, #000 80%, #000, #0000, #0000 90%, #000 90%, #000 )
    }
    resize-handle[bottom][left] { bottom: 1px; left: 1px; cursor: sw-resize; }
    resize-handle[bottom][left]:after {
      background: linear-gradient( 45deg, #000 10%, #000, #0000, #0000 20%, #000 20%, #000, #0000, #0000 30%, #000 30%, #000, #0000, #0000 40%, #000 40%, #000, #0000, #0000 50% )
    }
    resize-handle[bottom][right] { bottom: 1px; right: 1px; cursor: se-resize; }
    resize-handle[bottom][right]:after {
      background: linear-gradient( -45deg, #000 10%, #000, #0000, #0000 20%, #000 20%, #000, #0000, #0000 30%, #000 30%, #000, #0000, #0000 40%, #000 40%, #000, #0000, #0000 50% )
    }`;

    constructor(...args) {
        const self = super(...args);
        this.s = {};// start properties
        this.resizeFn = this.resize.bind(this);
        this.endResizeFn = this.endResize.bind(this);
        this.isTouchDevice = (('ontouchstart' in window) || (navigator.maxTouchPoints > 0) || (navigator.msMaxTouchPoints > 0));
        return self;
    }

    connectedCallback() {
        // check if parent position is relative or absolute.
        this.left = this.getAttribute('left') !== null;
        this.right = this.getAttribute('right') !== null;
        this.top = this.getAttribute('top') !== null;
        this.bottom = this.getAttribute('bottom') !== null;
        this.single = this.getAttribute('single') !== null;
        this.resizeEl = this.parentElement;
        this.containerEl = this.resizeEl.parentElement;

        addCss(this, ResizeHandle.css);
        this.addEventListener('mousedown', event => {
            this.startResize(event);
            document.body.addEventListener('mousemove', this.resizeFn);
            document.body.addEventListener('mouseup', this.endResizeFn);
            document.body.addEventListener('mouseleave', this.endResizeFn);
        });
        this.addEventListener('touchstart', event => {
            this.startResize(event);
            document.body.addEventListener('touchmove', this.resizeFn);
            document.body.addEventListener('touchend', this.endResizeFn);
        });
    }

    disconnectedCallback() {
        removeCss(this);
    }

    startResize(event) {
        disableBodyScroll(document.body);
        document.body.style.userSelect = 'none';

        const rbcr = this.resizeEl.getBoundingClientRect();
        const { clientX, clientY } = this.isTouchDevice ? event.changedTouches[0] : event;
        [this.s.x, this.s.y] = [clientX, clientY];
        [this.s.w, this.s.h] = [rbcr.width, rbcr.height];

        this.s.sibling = this.bottom ?  // bottom or top
            this.resizeEl.nextElementSibling : this.resizeEl.previousElementSibling;

        if (this.s.sibling) {
            const sbcr = this.s.sibling.getBoundingClientRect();
            [this.s.siblingW, this.s.siblingH] = [sbcr.width, sbcr.height];
        }

        this.containerEl = this.resizeEl.parentElement;
        if (this.containerEl) {
            this.s.containerSibling = this.right ? // right or left
                this.containerEl.nextElementSibling : this.containerEl.previousElementSibling;
            this.s.containerW = this.containerEl.getBoundingClientRect().width;
            this.s.containerSiblingW = this.s.containerSibling?.getBoundingClientRect().width;
        }
    }

    resize(event) {
        const { clientX, clientY } = this.isTouchDevice ? event.changedTouches[0] : event;
        let [dx, dy] = [clientX - this.s.x, clientY - this.s.y];
        this.left && (dx = -dx);
        this.top && (dy = -dy);

        if (this.single) {
            this.resizeEl.style.width = Math.max(this.s.w + dx, 64) + 'px';
            this.resizeEl.style.height = Math.max(this.s.h + dy, 32) + 'px';
        } else if (this.containerEl) {
            // update my width / height
            this.resizeEl.style.width = '100%';
            this.resizeEl.style.height = Math.max(this.s.h + dy, 32) + 'px';

            // update my sibling width / height
            if (this.s.sibling) {
                this.s.sibling.style.width = '100%';
                this.s.sibling.style.height = Math.max(this.s.siblingH - dy, 32) + 'px';
            }

            // update container(sibling) width / height
            this.containerEl.style.width = (this.s.containerW + dx) + 'px';
            if (this.s.containerSibling) {
                this.s.containerSibling.style.width = (this.s.containerSiblingW - dx) + 'px';
            }
        }
    }

    endResize(event) {
        enableBodyScroll(document.body);
        document.body.style.userSelect = '';
        document.body.removeEventListener('mousemove', this.resizeFn);
        document.body.removeEventListener('mouseup', this.endResizeFn);
        document.body.removeEventListener('mouseleave', this.endResizeFn);
        document.body.removeEventListener('touchmove', this.resizeFn);
        document.body.removeEventListener('touchend', this.endResizeFn);
    }
}

function addCss(el, css) {
    const tagName = el.tagName.toLowerCase();
    if (!document.querySelector(`style[${tagName}]`)) {
        const styleEl = document.createElement('style');
        styleEl.setAttribute(tagName, '');
        styleEl.appendChild(document.createTextNode(css));
        document.head.appendChild(styleEl);
    }
}

function removeCss(el) {
    const tagName = el.tagName.toLowerCase();
    const numXElements = document.body.querySelectorAll(`${tagName}`).length;
    const styleEl = document.querySelector(`style[${tagName}]`);
    (styleEl && numXElements < 1) && document.querySelector(`style[${tagName}]`).remove();
}